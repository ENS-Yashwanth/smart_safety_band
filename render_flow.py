from PIL import Image, ImageDraw, ImageFont

W, H = 1400, 1000
bg = (255, 255, 255)
img = Image.new('RGB', (W, H), color=bg)
d = ImageDraw.Draw(img)
font = ImageFont.load_default()

# Helper to draw rounded box with text
def draw_box(x, y, w, h, text, fill=(230, 244, 255), outline=(3, 54, 102)):
    r = 8
    d.rounded_rectangle([x, y, x+w, y+h], radius=r, fill=fill, outline=outline, width=2)
    try:
        bbox = d.textbbox((0,0), text, font=font)
        tw = bbox[2] - bbox[0]
        th = bbox[3] - bbox[1]
    except Exception:
        tw, th = font.getsize(text)
    d.text((x + (w-tw)/2, y + (h-th)/2), text, fill=(0,0,0), font=font)

# Helper to draw arrow
def draw_arrow(x1, y1, x2, y2, color=(0,0,0)):
    d.line([(x1, y1), (x2, y2)], fill=color, width=2)
    # arrowhead
    import math
    angle = math.atan2(y2-y1, x2-x1)
    ah = 12
    r1 = angle + 0.4
    r2 = angle - 0.4
    x3 = x2 - ah * math.cos(r1)
    y3 = y2 - ah * math.sin(r1)
    x4 = x2 - ah * math.cos(r2)
    y4 = y2 - ah * math.sin(r2)
    d.polygon([(x2,y2),(x3,y3),(x4,y4)], fill=color)

# Draw group boxes
d.rectangle([40, 30, W-40, 120], fill=(245,245,245), outline=(200,200,200))
d.text((60, 50), 'Flow Diagram: ESP32-S3 mini1 <-> SIM868', fill=(0,0,0), font=font)

# Host column (left)
hx = 100
hy = 150
col_w = 500
box_h = 60
vgap = 25

# Nodes in host
nodes_host = [
    ('Start: Device Power ON', hy),
    ('Create Cellular FreeRTOS Task', hy + (box_h+vgap)*1),
    ('Init GPIO power-key & UART', hy + (box_h+vgap)*2),
    ('Init PPP/ECM/RNDIS network stack', hy + (box_h+vgap)*6),
    ('Comms & Health Monitoring Loop', hy + (box_h+vgap)*7),
    ('Process TX queue / Handle RX frames', hy + (box_h+vgap)*8),
    ('Reconnection state machine (exp backoff)', hy + (box_h+vgap)*9),
    ('Shutdown modem gracefully', hy + (box_h+vgap)*11)
]
for i, (txt, y) in enumerate(nodes_host):
    draw_box(hx, y, col_w, box_h, txt)

# Modem column (right)
mx = hx + col_w + 140
nodes_modem = [
    ('Power on SIM868 (GPIO toggle)', hy + (box_h+vgap)*2),
    ('Poll modem with "AT" until OK', hy + (box_h+vgap)*3),
    ('Query SIM (AT+CPIN?)', hy + (box_h+vgap)*4),
    ('Configure APN (AT+CGDCONT)', hy + (box_h+vgap)*5),
    ('Trigger network attach (AT+COPS=0)', hy + (box_h+vgap)*6),
    ('Poll registration (AT+CEREG? / AT+CGREG?)', hy + (box_h+vgap)*7),
    ('Continuous signal monitoring (AT+CSQ / AT+CESQ)', hy + (box_h+vgap)*8)
]
for txt, y in nodes_modem:
    draw_box(mx, y, col_w, box_h, txt, fill=(255,244,230), outline=(184,92,0))

# Draw arrows between specific boxes to reflect the mermaid flow
# Host Start -> Create Task -> Init GPIO -> Power on
draw_arrow(hx+col_w/2, hy+box_h, hx+col_w/2, hy+box_h+vgap)
draw_arrow(hx+col_w/2, hy+(box_h+vgap)*2, mx+20, hy+(box_h+vgap)*2+box_h/2)
# Power on -> AT ping -> OK -> SIMCHK -> APN -> ATTACH -> REG
draw_arrow(mx+10, hy+(box_h+vgap)*2+box_h/2, mx+10, hy+(box_h+vgap)*3)
draw_arrow(mx+10, hy+(box_h+vgap)*3+box_h, mx+10, hy+(box_h+vgap)*4)
draw_arrow(mx+10, hy+(box_h+vgap)*4+box_h, mx+10, hy+(box_h+vgap)*5)
draw_arrow(mx+10, hy+(box_h+vgap)*5+box_h, mx+10, hy+(box_h+vgap)*6)
draw_arrow(mx+10, hy+(box_h+vgap)*6+box_h, mx+10, hy+(box_h+vgap)*7)

# REG -> NET_INIT (modem to host)
draw_arrow(mx, hy+(box_h+vgap)*7+box_h/2, hx+col_w-10, hy+(box_h+vgap)*6+box_h/2)
# NET_INIT -> SIGNAL -> MON_LOOP
draw_arrow(hx+col_w/2, hy+(box_h+vgap)*6+box_h, hx+col_w/2, hy+(box_h+vgap)*7)
# MON_LOOP -> TXRX -> back to MON_LOOP
draw_arrow(hx+col_w/2, hy+(box_h+vgap)*7+box_h, hx+col_w/2, hy+(box_h+vgap)*8)
draw_arrow(hx+col_w/2, hy+(box_h+vgap)*8+box_h, hx+col_w/2, hy+(box_h+vgap)*7)

# Reconnection flow: REG drop -> RECON -> ATTACH
# Draw an arrow from REG down-right to recon box
reg_x = mx
reg_y = hy+(box_h+vgap)*7+box_h/2
recon_x = hx + col_w/2
recon_y = hy + (box_h+vgap)*9 + box_h/2
draw_arrow(reg_x, reg_y, recon_x+40, recon_y)
draw_arrow(recon_x+40, recon_y+10, mx+20, hy+(box_h+vgap)*6+10)  # back to ATTACH

# Save PNG
img.save('flow_diagram.png')
print('Saved flow_diagram.png')
