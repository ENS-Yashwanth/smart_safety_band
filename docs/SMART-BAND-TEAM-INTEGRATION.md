# Smart Band integration handoff

This is the HTTP integration profile for the Smart Band vendor team. The band
or vendor gateway produces three canonical events: `device.heartbeat`,
`sos.triggered`, and `sos.updated`. ICCC creates an incident only from the first
accepted `sos.triggered` event.

The machine-readable contract is
[`schemas/smart-band-events.schema.json`](schemas/smart-band-events.schema.json).

## Endpoint

Send each complete JSON event with:

```http
POST /api/ingestion/api/v1/events/normalized HTTP/1.1
Content-Type: application/json
```

That path is the Command Center proxy used by the demo. With the local demo
port-forward, the full URL is:

```text
http://127.0.0.1:8083/api/ingestion/api/v1/events/normalized
```

Inside Kubernetes, the adapter uses:

```text
http://event-ingestion:8080/api/v1/events/normalized
```

The public HTTPS base URL and authentication mechanism are not implemented yet
and must be agreed before production. Do not integrate a physical device with
the `/simulate/*` endpoints; those control only the fake band.

A successful response wraps the complete accepted event (abbreviated here):

```json
{"accepted":true,"duplicate":false,"event":{"schema_version":"smart-city-event/1.0"}}
```

Retry a failed request with the same `event_id`. Uniqueness is the pair
`source.source_id + event_id`; a retry returns `duplicate: true` and does not
create another incident.

```bash
curl --fail-with-body -X POST \
  -H 'Content-Type: application/json' \
  --data @smart-band-event.json \
  'http://127.0.0.1:8083/api/ingestion/api/v1/events/normalized'
```

Before vendor testing, the ICCC team must configure the adapter with the stable
band ID. Adapter startup registers the device and provisions the critical
`sos.triggered` alert rule. Posting an event for an unprovisioned band may store
the analytics event but will not automatically create the intended incident.

## Audio delivery

Do not put audio bytes, chunks, or base64 in JSON. The current receiver has no
multipart or raw-audio upload endpoint.

1. The band streams its vendor-specific audio packets to the vendor gateway.
2. The gateway assembles one complete short clip and exposes it through an
   HTTP(S) `GET` URL reachable by the `event-ingestion` Pod.
3. The gateway sends `sos.triggered` with that URL in `evidence[0].uri`.
4. ICCC downloads the clip synchronously, computes SHA-256, stores it in MinIO,
   and replaces the URL with `evidence:<sha256>` in the accepted event.

The gateway may provide mono PCM WAV (`audio/wav`) or a complete AMR/AMR-WB
artifact (`audio/amr` or `audio/amr-wb`). ICCC transcodes AMR input to mono,
16 kHz, 16-bit PCM WAV with `ffmpeg` before hashing and storing the WAV in
MinIO. The accepted event therefore exposes `evidence[].content_type` as
`audio/wav`; `payload.audio` continues to describe the vendor artifact that was
submitted.

Keep both the incoming object and converted WAV below 8 MiB. Return the correct
`Content-Type`, require no browser cookies or custom headers, and keep a
presigned URL valid for at least five minutes. If the vendor cannot provide a
retrievable URL, a separate authenticated ICCC audio-upload endpoint must be
implemented; it does not exist today.

For a new critical SOS notification, the Command Center places a persistent
operator-attention card in the right sidebar and attempts to play the retained
WAV once. Browsers may block audible autoplay before the operator has interacted
with the page, so the card and incident detail both keep a visible Play control.
The browser retrieves audio only through the ICCC evidence endpoint; it never
uses the vendor URL after MinIO archival.

## Identity and episode rules

- `source.source_id` and `payload.band_id` must be the same stable band ID.
- Use UTC RFC 3339 timestamps ending in `Z`.
- Omit `received_at`; ICCC assigns it when the event is accepted.
- Generate a unique, retry-stable `event_id` for every emitted event.
- Generate one new `correlation_id` when SOS begins. Reuse it for the trigger
  and every location update in that episode.
- Use `device:<band_id>` as the heartbeat correlation ID.
- Send heartbeats every 10 seconds and location updates every 5 seconds during
  an active SOS unless another interval is agreed.
- Latitude is -90..90, longitude is -180..180, and `accuracy_m` is non-negative.

## 1. Heartbeat

```json
{
  "schema_version": "smart-city-event/1.0",
  "event_id": "band-demo-001-heartbeat-018f45f8-51f2-7a3d-9c45-92f848c73001",
  "correlation_id": "device:band-demo-001",
  "event_type": "device.heartbeat",
  "severity": "info",
  "occurred_at": "2026-09-01T04:14:28.685830Z",
  "source": {
    "kind": "smart-band",
    "source_id": "band-demo-001",
    "deployment_id": null
  },
  "location": {
    "kind": "geo",
    "site_id": "hyderabad-demo",
    "zone_id": null,
    "latitude": 17.385044,
    "longitude": 78.486671,
    "accuracy_m": 8.0
  },
  "evidence": [],
  "payload": {
    "band_id": "band-demo-001",
    "status": "online",
    "last_seen": "2026-09-01T04:14:28.685830Z",
    "battery_percent": 86
  }
}
```

Use the same event with `status: "offline"` for a graceful disconnect. ICCC
also determines offline state when heartbeats stop for the agreed timeout.

## 2. SOS triggered

```json
{
  "schema_version": "smart-city-event/1.0",
  "event_id": "band-demo-001-sos-018f45f8-6db3-7fa3-bc29-e22fd9c93002",
  "correlation_id": "sos-018f45f8-6dad-7ace-9052-b98ca7b73000",
  "event_type": "sos.triggered",
  "severity": "critical",
  "occurred_at": "2026-09-01T04:15:00.000000Z",
  "source": {
    "kind": "smart-band",
    "source_id": "band-demo-001",
    "deployment_id": null
  },
  "location": {
    "kind": "geo",
    "site_id": "hyderabad-demo",
    "zone_id": null,
    "latitude": 17.385044,
    "longitude": 78.486671,
    "accuracy_m": 8.0
  },
  "evidence": [
    {
      "type": "audio",
      "uri": "https://vendor.example/evidence/audio-018f45f8.amr?signature=REDACTED",
      "content_type": "audio/amr"
    }
  ],
  "payload": {
    "band_id": "band-demo-001",
    "trigger": "button",
    "sos_status": "triggered",
    "location_source": "gps",
    "altitude_m": null,
    "audio": {
      "evidence_id": "audio-018f45f8",
      "content_type": "audio/amr",
      "duration_seconds": 1.5,
      "size_bytes": 24044
    }
  }
}
```

Send this once per button activation. Re-send the same `event_id` on uncertain
delivery; never create a second trigger event merely because the HTTP response
was lost.

## 3. SOS location update

```json
{
  "schema_version": "smart-city-event/1.0",
  "event_id": "band-demo-001-location-018f45f8-8120-7137-a65a-8d92f3e93003",
  "correlation_id": "sos-018f45f8-6dad-7ace-9052-b98ca7b73000",
  "event_type": "sos.updated",
  "severity": "critical",
  "occurred_at": "2026-09-01T04:15:05.000000Z",
  "source": {
    "kind": "smart-band",
    "source_id": "band-demo-001",
    "deployment_id": null
  },
  "location": {
    "kind": "geo",
    "site_id": "hyderabad-demo",
    "zone_id": null,
    "latitude": 17.385124,
    "longitude": 78.486731,
    "accuracy_m": 7.0
  },
  "evidence": [],
  "payload": {
    "band_id": "band-demo-001",
    "sos_status": "active",
    "update_type": "location",
    "sequence": 1,
    "location_source": "gps",
    "altitude_m": null
  }
}
```

Increment `sequence` for every location update and preserve the SOS
`correlation_id`. If the band supports cancellation, emit `sos.cancelled` with
that same correlation ID, the last coordinates, and a short cancellation
reason. Cancellation is recorded in the audit trail; the operator still closes
the incident according to ICCC policy.

## Vendor handoff checklist

- Provide the real gRPC `.proto` and authentication/TLS requirements; ICCC will
  implement `GrpcSmartBandTransport` from that source rather than guess messages.
- Confirm band ID format, heartbeat/offline timeout, GPS cadence and accuracy.
- Confirm audio codec, sample rate, maximum duration, packet ordering, and how
  the gateway exposes the completed clip URL.
- Test retry with the same event ID, invalid coordinates, expired audio URL,
  SOS updates with one correlation ID, and SOS cancellation.
