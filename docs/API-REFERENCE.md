# Implemented API reference

The demo runs four FastAPI services. Each service exposes interactive OpenAPI
at `/docs` and machine-readable OpenAPI at `/openapi.json`. No authentication,
authorization, or TLS is implemented; expose these APIs only in the controlled
demo network. JSON request models reject unknown fields unless stated otherwise.

Default local ports are platform `8081`, ingestion `8082`, command center
`8083`, and the smart-band adapter `8084`. Examples below use
`http://127.0.0.1:<port>`.

## 1. Platform API (`:8081`)

### Catalog and deployment endpoints

| Method and path | Request | Success | Important errors |
|---|---|---|---|
| `GET /api/v1/catalog/solutions` | — | Catalog entries | — |
| `GET /api/v1/catalog/solutions/{catalog_id}` | — | Catalog entry | 404 |
| `POST /api/v1/catalog/refresh` | — | Refreshed entries and image resolution status | Registry failures are recorded per entry |
| `GET /api/v1/deployments` | — | `DeploymentRequest[]` | — |
| `GET /api/v1/deployments/{id}` | — | `DeploymentRequest` | 404 |
| `PUT /api/v1/deployments/{id}` | `DeploymentRequest` | Stored request after Kubernetes apply | 400 ID mismatch; 409 revision/conflict; 422 catalog/image; 503 cluster |
| `DELETE /api/v1/deployments/{id}` | — | 204 | 404; 409 catalog/change; 503 cluster |
| `POST /api/v1/deployments/{id}/actions/start` | — | Updated request | 404/422/503 |
| `POST /api/v1/deployments/{id}/actions/stop` | — | Updated request | 404/422/503 |
| `GET /api/v1/deployments/{id}/resource` | — | Rendered `SolutionDeployment` CR | 404 |

`DeploymentRequest`:

```json
{
  "deployment_id": "corner-a-surveillance",
  "catalog_id": "surveillance-edge-runtime:2026.08.24-v3",
  "revision": 1,
  "desired_lifecycle": "Running",
  "desired_state": {
    "edge_id": "edge-285h",
    "revision": 1,
    "cameras": [{
      "camera_id": "camera-a",
      "source": "file:/run/secrets/apexfabric/camera-a.rtsp",
      "solution_pack": "surveillance",
      "fps": 8,
      "apps": ["intrusion"],
      "config": {"zones": {"intrusion": [{"name": "restricted", "poly": [[0,0],[1,0],[1,1]]}]}}
    }]
  },
  "secrets": {"camera-a.rtsp": "rtsp://user:password@camera/stream"}
}
```

`deployment_id` must be a lowercase DNS-style name. `revision` starts at 1 and
every update—including start/stop—must increment exactly by one. Lifecycle is
`Running` or `Stopped`. The catalog image must be resolved to an immutable
digest. Catalog loading checks each JSON Schema and validates its checked-in
example, but the current deployment PUT path does **not** validate submitted
`desired_state` against that schema. Integrators must validate it before
submission until that API gap is closed. Current schemas and examples live
under `solution-packs/catalog/<pack>/`.

The request's `secrets` keys become projected secret files; the desired state
must refer to them as `file:/run/secrets/apexfabric/<key>`. Never send secrets
in an event or put credentials directly in desired state.

### Device and edge-box endpoints

| Method and path | Request | Success | Important errors |
|---|---|---|---|
| `GET /api/v1/devices` | — | `Device[]` | — |
| `POST /api/v1/devices` | `Device` | 201 + device | 409 duplicate ID |
| `PUT /api/v1/devices/{id}` | `Device` | Upserted device | 400 ID mismatch |
| `DELETE /api/v1/devices/{id}` | — | 204 | 404 |
| `GET /api/v1/devices/{id}/snapshot` | — | Fresh `image/jpeg`, max 8 MiB | 404/413/502/503 |
| `GET /api/v1/boxes` | — | `EdgeBox[]` | — |
| `PUT /api/v1/boxes/{id}` | `EdgeBox` | Upserted box | 400 ID mismatch |

```json
{
  "device_id": "camera-a",
  "kind": "camera",
  "display_name": "North Gate Camera",
  "site_id": "hyderabad-demo",
  "enabled": true,
  "configuration": {"rtsp_url": "rtsp://user:password@camera/stream"}
}
```

Device kinds are `camera`, `smart-band`, `display`, and `drone`. A device ID is
lowercase DNS-style. `configuration` is vendor-specific and currently not
schema-validated. Camera snapshot capture requires `ffmpeg` and a configured
`rtsp_url`; it tries TCP then UDP and returns `Cache-Control: no-store`.

```json
{
  "box_id": "edge-285h-a",
  "display_name": "Intel 285H Edge",
  "node_name": "edge-285h",
  "hardware_profile": "intel-285h",
  "architecture": "amd64",
  "status": "online",
  "qualified": true,
  "site_id": "hyderabad-demo",
  "capabilities": {"decoder": "vaapi"}
}
```

Architecture is `amd64` or `arm64`; status is `online`, `offline`, or `unknown`.

### Kubernetes inventory

`GET /api/v1/kubernetes` returns connection status plus `nodes`, `workloads`,
`pods`, and `services`. The four collection endpoints are also available at
`/api/v1/kubernetes/{nodes|workloads|pods|services}`. Their response objects are
operational snapshots, not stable vendor contracts; use the platform resource
and deployment APIs for integrations.

## 2. Ingestion API (`:8082`)

| Method and path | Request/query | Success | Important errors |
|---|---|---|---|
| `POST /api/v1/events` | Vendor analytics object | Acceptance envelope | 422 |
| `POST /api/v1/events/normalized` | Canonical event | Acceptance envelope | 422 |
| `GET /api/v1/events` | `limit=1..1000`, optional `source_id`, `deployment_id` | Canonical events, newest first | 422 limit |
| `GET /api/v1/collectors` | — | Collector status array | — |
| `GET /api/v1/events/{source_id}/{event_id}/evidence/{index}` | — | Snapshot/audio evidence bytes; accepted AMR audio is returned as archived WAV | 404/422/502 |
| `GET /api/v1/storage` | — | SQLite/MinIO usage and limits | — |

See [EVENT-CONTRACTS.md](EVENT-CONTRACTS.md) for complete request schemas,
normalization, SSE, evidence, and deduplication behavior.

## 3. Command Center API (`:8080`)

The command center also proxies `/api/platform/{path}` to the platform service,
`/api/ingestion/{path}` to ingestion, and `/api/smart-band/{path}` to the fake
band adapter. Proxy paths include the upstream path. These convenience routes
are not in OpenAPI.

### Rules, events, notifications, and incidents

| Method and path | Request | Success | Important errors |
|---|---|---|---|
| `GET /api/v1/alert-rules` | — | `AlertRule[]` | — |
| `PUT /api/v1/alert-rules/{uuid}` | `AlertRule` | Stored rule | 400 ID mismatch; 422 model |
| `DELETE /api/v1/alert-rules/{uuid}` | — | 204 | 404 |
| `POST /api/v1/internal/events` | Canonical event | Newly created `Incident[]` | 422 |
| `GET /api/v1/notifications` | `limit=1..1000`, `unread_only` | Notifications, newest first | 422 |
| `POST /api/v1/notifications/{uuid}/read` | — | Updated notification | 404 |
| `GET /api/v1/incidents` | — | `Incident[]` | — |
| `GET /api/v1/incidents/{uuid}` | — | Incident detail with events/actions/timeline/audit | 404 |
| `POST /api/v1/incidents/{uuid}/acknowledge` | `{"note":"..."}` | Incident detail | 404/409/422 |
| `POST /api/v1/incidents/{uuid}/resolve` | `{"note":"..."}` | Incident detail | 404/409/422 |
| `POST /api/v1/incidents/{uuid}/close` | `{"note":"..."}` | Incident detail | 404/409/422 |
| `POST /api/v1/incidents/{uuid}/notes` | `{"note":"..."}` | Incident detail | 404/422 |

`OperatorNote.note` is required and 1–2000 characters.

An alert rule example:

```json
{
  "rule_id": "7ae637d7-c582-4240-b7a5-5dd3e48d1310",
  "name": "Crowd threshold",
  "application": "people_counting",
  "metric_field": "payload.count",
  "metric_threshold": 10,
  "enabled": true,
  "filter": {
    "event_types": ["crowd.threshold_exceeded"],
    "deployment_ids": ["corner-a-surveillance"],
    "severities": [], "source_ids": [], "zone_ids": []
  },
  "threshold": 1,
  "window_seconds": 60,
  "cooldown_seconds": 60,
  "incident_severity": "warning",
  "actions": ["notification"],
  "notification_routes": ["dashboard"],
  "route_configuration": {}
}
```

`event_types` must be non-empty. Empty filter sets mean “any.” The optional
metric field is a dot-separated path and matches when its numeric value is at
least `metric_threshold`. `threshold` matching events must arrive within
`window_seconds`; the rule then observes `cooldown_seconds`. Actions are
`display` or `notification`. Notification routes are `dashboard`, `voice-call`,
`display-board`, or `webhook`; only dashboard notification creation and the
simulator action adapter are implemented today.

Incident states are `Open`, `Acknowledged`, `ActionPending`, `ActionActive`,
`ActionFailed`, and `Resolved`.

### Actions and vendor callbacks

| Method and path | Request | Success | Important errors |
|---|---|---|---|
| `POST /api/v1/incidents/{uuid}/actions/approve` | `ActionDecision` | Incident detail | 404/409/422 |
| `POST /api/v1/incidents/{uuid}/actions/reject` | `ActionDecision` | Incident detail | 404/409/422 |
| `POST /api/v1/incidents/{uuid}/actions` | `ActionRequest` | Incident, action, authorizer | 404/409/422 |
| `POST /api/v1/actions/{action_id}/callbacks` | `ActionCallback` | Incident detail | 404/409/422 |

```json
{
  "action_type": "drone",
  "note": "Approved by incident commander; weather and airspace checked",
  "payload": {"destination": {"latitude": 17.385, "longitude": 78.4867}}
}
```

`ActionDecision.action_type` is `display`, `notification`, or `drone`; it
defaults to `notification`. Approval is restricted to critical incidents in
`Open`, `Acknowledged`, or `ActionFailed`. The direct `ActionRequest` form also
requires a non-empty caller-supplied `idempotency_key` and can only be submitted
from `Acknowledged` or `ActionFailed`. For the demo flow, display and
notification actions complete the incident once the command center records the
approved downstream effect; drone incidents stay active until the docked
callback arrives.

Vendor callback:

```json
{
  "state": "active",
  "provider_event_id": "vendor-event-8842",
  "vendor_reference": "mission-42",
  "detail": {"latitude": 17.3849, "longitude": 78.4865, "battery_percent": 91}
}
```

Callback states are `accepted`, `active`, `succeeded`, `failed`, `completed`,
and `docked`. `failed` moves an active/pending incident to `ActionFailed`;
accepted/active moves pending to active; display/notification actions resolve
once the action record is completed; drone missions remain active through
`succeeded` and `completed` and only resolve on `docked`. Callback
authentication and duplicate provider-event enforcement are not implemented and
must be added for a real vendor connection.

## 4. Fake Smart Band Adapter (`:8084`)

| Method and path | Request | Success | Important errors |
|---|---|---|---|
| `GET /simulate/status` | — | Online state, last seen, active correlation, current coordinates | — |
| `POST /simulate/start` | — | Registration, SOS rule setup, initial heartbeat | 503 dependency |
| `POST /simulate/sos` | Optional coordinate override; optional `Idempotency-Key` header | Canonical SOS and ingestion result | 409 not started; 422 coordinates; 503 dependency |
| `POST /simulate/location` | Required coordinates | Canonical update and ingestion result | 409 no episode; 422 coordinates |
| `POST /simulate/cancel` | Optional `{"reason":"..."}` | Canonical cancellation and ingestion result | 409 no episode |
| `POST /simulate/stop` | — | Offline heartbeat and stopped status | 503 dependency |
| `GET /evidence/{evidence_id}` | — | Generated `audio/wav` | 404 |

Use the command-center proxy forms such as
`POST http://127.0.0.1:8083/api/smart-band/simulate/sos` when only the single
demo port-forward is running. Complete controls and exact event examples are in
[SMART-BAND-SIMULATOR.md](SMART-BAND-SIMULATOR.md).

Physical-band and vendor-gateway integrations must not use `/simulate/*`.
Their implemented HTTP output boundary, audio evidence flow, retry rules, exact
JSON examples, and machine-readable schema are documented in
[SMART-BAND-TEAM-INTEGRATION.md](SMART-BAND-TEAM-INTEGRATION.md).

## 5. Common HTTP conventions

- JSON requests use `Content-Type: application/json`; snapshot responses are binary.
- FastAPI model validation failures return HTTP 422 with a `detail` array.
- 204 responses have no body. 404 means the resource is unknown.
- 409 means identity, revision, state-transition, or concurrency conflict.
- 503 means an upstream service, Kubernetes, or required capture capability is unavailable.
- CORS currently permits all origins, methods, and headers; this is demo-only.
- `GET /healthz` on every service returns `{"status":"ok"}` and is omitted from OpenAPI.
