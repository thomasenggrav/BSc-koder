import time
import random
import serial
from datetime import datetime

# Innstillinger
PORT = "COM5"
BAUD = 115200

# Timing for Signature500 OK
SIG500_OK_DELAY_S = 3.0

# Timing for Seabird
TPS_DELAY_BEFORE_SEND_S = 0.2

# Wakeup sekvens til signature 500
SIG500_WAKE_SEQUENCE = ["@@@@@@", "K1W%!Q", "K1W%!Q"]
SIG500_WAKE_PART_TIMEOUT_S = 1.0
SEND_SIG500_WAKE_OK = True

# Timing for Signature500 etter START
SIG500_DELAY_BEFORE_OK_S = 2.0
SIG500_DELAY_AFTER_OK_BEFORE_BURST_S = 3.0
SEND_SIG500_START_OK = True

# Små mellomrom mellom linjene i SIG500 pakkene
SIG500_INTERLINE_DELAYS_S = [0.040, 0.040, 0.040, 0.040]

# Cell-posisjoner for de 3 PNORC-linjene
SIG500_CELL_POSITIONS = [2.5, 4.5, 6.5]

# Statuskode i PNORH
SIG500_STATUS_CODE = "2A4C0000"

# Maks linjelengde for buffer til kommando
MAX_CMD_LINE_LEN = 128

# Startverdier for Seabird
seabird_temp = 7.2345
seabird_cond = 4.1234
seabird_pres = 452.345
seabird_sal = 34.5678
seabird_spcond = 42.1234

# Startverdier for Signature500
sig_ec = 0
sig_bv = 22.9
sig_ss = 1500.0
sig_h = 123.4
sig_pi = 45.6
sig_r = -5.3
sig_p = 705.658
sig_t = 24.56

# Holder for planlagte signature 500 events
scheduled_sig500_events = []

# Wakeup-parser state
sig500_wake_stage = 0
sig500_wake_pos = 0
sig500_last_wake_byte_ts = 0.0

# Holde verdier innenfor max og min
def clamp(x, lo, hi):
    return max(lo, min(hi, x))

# Sjekksum for NMEA pakke
def nmea_checksum(payload: str) -> str:
    cs = 0
    for ch in payload:
        cs ^= ord(ch)
    return f"{cs:02X}"

# Lager NMEA setning med $ + sjekksum + linjeskift/carriage retur
def lag_nmea_setning(payload: str) -> str:
    return f"${payload}*{nmea_checksum(payload)}\r\n"

# Sende ASCII tekst over serial
def send_ascii(ser, text: str):
    ser.write(text.encode("ascii"))
    ser.flush()

# Sende linjer med små mellomrom
def send_lines_with_delays(ser, lines, interline_delays_s=None):
    if interline_delays_s is None:
        interline_delays_s = []

    for i, line in enumerate(lines):
        send_ascii(ser, line)

        if i < len(lines) - 1:
            if i < len(interline_delays_s):
                time.sleep(interline_delays_s[i])

# Nullstilling av wakeup parsingen
def reset_sig500_wake_parser():
    global sig500_wake_stage, sig500_wake_pos, sig500_last_wake_byte_ts
    sig500_wake_stage = 0
    sig500_wake_pos = 0
    sig500_last_wake_byte_ts = 0.0

# Seabird-simulering
def oppdater_seabird():
    global seabird_temp, seabird_cond, seabird_pres, seabird_sal, seabird_spcond

    # Små variasjoner i forventet verdier
    seabird_temp = clamp(seabird_temp + random.uniform(-0.03, 0.03), -2.0, 15.0)
    seabird_cond = clamp(seabird_cond + random.uniform(-0.01, 0.01), 3.0, 5.5)
    seabird_pres = clamp(seabird_pres + random.uniform(-0.10, 0.10), 400.0, 500.0)
    seabird_sal = clamp(seabird_sal + random.uniform(-0.02, 0.02), 32.0, 37.0)
    seabird_spcond = clamp(seabird_spcond + random.uniform(-0.05, 0.05), 30.0, 60.0)

# Lager en komplett seabird pakke
def bygg_seabird_linje(now: datetime) -> str:
    oppdater_seabird()

    dato_tid = now.strftime("%d %b %Y, %H:%M:%S")
    return (
        f"#{seabird_temp:7.4f},   " # 7 tegn, 4 desimaler
        f"{seabird_cond:7.4f},    "
        f"{seabird_pres:7.3f},    "
        f"{seabird_sal:7.4f},     "
        f"{seabird_spcond:7.4f},  "
        f"{dato_tid}\r\n"
    )

# Signature500-simulering
def oppdater_sig500():
    global sig_bv, sig_ss, sig_h, sig_pi, sig_r, sig_p, sig_t

    # Legger til små variasjoner
    sig_bv = clamp(sig_bv + random.uniform(-0.05, 0.05), 0.0, 30.0)
    sig_ss = clamp(sig_ss + random.uniform(-2.0, 2.0), 1300.0, 1700.0)
    sig_h = (sig_h + random.uniform(-0.5, 0.5)) % 360.0
    sig_pi = clamp(sig_pi + random.uniform(-0.5, 0.5), -90.0, 90.0)
    sig_r = clamp(sig_r + random.uniform(-0.5, 0.5), -180.0, 180.0)
    sig_p = clamp(sig_p + random.uniform(-0.05, 0.05), 500.0, 900.0)
    sig_t = clamp(sig_t + random.uniform(-0.05, 0.05), -2.0, 40.0)

# Lage en komplett signature 500 pakke med 1 header, 1 sensor og 3 celle
def bygg_sig500_burst(now: datetime):
    global sig_ec

    oppdater_sig500()

    # øker counter for hver pakke
    sig_ec = (sig_ec + 1) % 1000000

    dato = now.strftime("%m%d%y")
    klokke = now.strftime("%H%M%S")
    lines = []

    # lage en PNORH linje
    lines.append(lag_nmea_setning(f"PNORH4,{dato},{klokke},{sig_ec},{SIG500_STATUS_CODE}"))

    # Lage en PNORS linje
    lines.append(lag_nmea_setning(f"PNORS4,{sig_bv:.1f},{sig_ss:.1f},{sig_h:.1f},{sig_pi:.1f},{sig_r:.1f},{sig_p:.3f},{sig_t:.2f}"))

    # lage 3 PNORC linjer
    for cp in SIG500_CELL_POSITIONS:
        sp = clamp(random.uniform(0.0, 2.0), 0.0, 5.0)
        direction = random.uniform(0.0, 359.99)
        ac = random.randint(0, 9)
        aa = random.randint(0, 50)

        lines.append(lag_nmea_setning(f"PNORC4,{cp:.1f},{sp:.3f},{direction:.2f},{ac},{aa}" ))

    return lines

# Wakeup parsing 
def prosesser_sig500_wakeup_byte(ser, ch: str) -> bool:
    # Returnerer True hvis byten ble tatt som en del av wakeup sekvensen, false hvis ikke

    global sig500_wake_stage, sig500_wake_pos, sig500_last_wake_byte_ts

    # Sjekk timeout for wakeup sekvens
    now_mono = time.monotonic()

    # Timeout mellom bytes i wake-sekvens, følge en bestemt tid mellom hver byte, hvis ikke reset
    if (sig500_wake_stage != 0 or sig500_wake_pos != 0):
        if (now_mono - sig500_last_wake_byte_ts) > SIG500_WAKE_PART_TIMEOUT_S:
            print("[WAKE] Timeout i wakeup-sekvens -> reset")
            reset_sig500_wake_parser()

    # Starte bare hvis første tegn stemmer
    if sig500_wake_stage == 0 and sig500_wake_pos == 0:
        if ch != SIG500_WAKE_SEQUENCE[0][0]:
            return False

    # Henter forventet sekvens for nåværende stage
    expected = SIG500_WAKE_SEQUENCE[sig500_wake_stage]

    # Sjekke om mottatt byte matcher forventet byte
    if ch == expected[sig500_wake_pos]:
        sig500_wake_pos += 1
        # Oppadeter timestamp
        sig500_last_wake_byte_ts = now_mono

        # Dersom hele byte er mottatt, gå videre til neste stage
        if sig500_wake_pos >= len(expected):
            print(f"[WAKE] MOTTATT del {sig500_wake_stage + 1}/{len(SIG500_WAKE_SEQUENCE)}: {expected}")
            sig500_wake_stage += 1
            sig500_wake_pos = 0

            # Dersom hele sekvensen er mottatt, returnere OK
            if sig500_wake_stage >= len(SIG500_WAKE_SEQUENCE):
                print("[WAKE] Full Signature500 wakeup mottatt")
                if SEND_SIG500_WAKE_OK:
                    time.sleep(SIG500_OK_DELAY_S)
                    send_ascii(ser, "OK\r\n")
                    print("SEND: OK (wake)")
                reset_sig500_wake_parser()

        return True
    
    # DErsom tegnet ikke stemmet med forventet, forkastes wakeup sekvens
    print("[WAKE] Mismatch i wakeup-sekvens -> reset")
    reset_sig500_wake_parser()

    return False

# Kommandohåndtering
def håndter_kommando(ser, cmd: str):
    global scheduled_sig500_events

    # Fjerne mellomrom, og sette den i blokk bokstaver
    cmd = cmd.strip().upper()

    if not cmd:
        return

    # Mottatt data fra sjøbunnsenheten
    print(f"MOTTATT: {cmd}")

    # Dersom TPS mottas, printe ut seabird data
    if cmd == "TPS":
        print(f"[SEABIRD] Venter {TPS_DELAY_BEFORE_SEND_S:.3f} s før svar på TPS")
        time.sleep(TPS_DELAY_BEFORE_SEND_S)

        now = datetime.now()
        line = bygg_seabird_linje(now)
        send_ascii(ser, line)
        print("SEND:", line.strip())

    # Dersom kommandoen er START, sende ut Signature 500
    elif cmd == "START":
        now_mono = time.monotonic()

        # Tid for når OK skal sendes, samt resterende data
        ok_due = now_mono + SIG500_DELAY_BEFORE_OK_S
        burst_due = ok_due + SIG500_DELAY_AFTER_OK_BEFORE_BURST_S

        # Planlagt hendelser
        scheduled_sig500_events.append({
            "ok_due": ok_due,
            "burst_due": burst_due,
            "ok_sent": False,
        })

        # Total delay på pakken
        total_delay = SIG500_DELAY_BEFORE_OK_S + SIG500_DELAY_AFTER_OK_BEFORE_BURST_S
        print(f"[SIG500] START mottatt -> OK om {SIG500_DELAY_BEFORE_OK_S:.1f} s, "f"burst om {total_delay:.1f} s")

    else:
        print(f"[IGNORERT] Ukjent kommando: {cmd}")


def prosesser_planlagte_send(ser):
    global scheduled_sig500_events
    
    # Dersom ingen planlagte hendelser, ikke sende noe og gå ut av funksjon
    if not scheduled_sig500_events:
        return
    
    # Monotonic tid (tid fra boot osv, teller bare fra en tid opp fra 0, ikke dato)
    now_mono = time.monotonic()
    remaining = []

    # GÅr igjennom alle planlagte events
    for event in scheduled_sig500_events:
        if SEND_SIG500_START_OK and (not event["ok_sent"]) and now_mono >= event["ok_due"]:
            send_ascii(ser, "OK\r\n")
            print("SEND: OK (START)")
            # DErsom den er gjennomført sette til True
            event["ok_sent"] = True

        # Dersom tiden er nådd skal datasettene sendes
        if now_mono >= event["burst_due"]:
            now = datetime.now()
            lines = bygg_sig500_burst(now)

            send_lines_with_delays(ser,lines,SIG500_INTERLINE_DELAYS_S)

            print("[SIG500] SEND burst:")
            for line in lines:
                print("SEND:", line.strip())
        else:
            remaining.append(event)

    scheduled_sig500_events = remaining


# Main loop
def main():
    # Åpne seriell tilkobling
    ser = serial.Serial(PORT, BAUD, timeout=0.05)
    print(f"Koblet til {PORT} @ {BAUD}")

    # Buffer som inneholder kommandoer
    line_buf = bytearray()

    try:
        # Looper for alltid
        while True:
            # Hvor mange bytes ligger klar
            waiting = ser.in_waiting
            # DErsom det er bytes klar
            if waiting > 0:
                # Lese disse
                data = ser.read(waiting)

                # Gå igjennom en byte om gangen
                for b in data:
                    try:
                        # Endre fra byte til char
                        ch = chr(b)

                    # Prøver først å prosessere som rå verdi
                    if prosesser_sig500_wakeup_byte(ser, ch):
                        continue

                    # Hvis ikke rå verdi går, prosessere som tekst
                    if b == 0x0D:   # \r
                        continue
                    # Lete etter slutten av kommandoen
                    if b == 0x0A:   # \n
                        # Kunn dekode dersom det er data i buffer
                        if line_buf:
                            try:
                                # Dekode og gjøre om fra bytes til ASCII
                                cmd = line_buf.decode("ascii", errors="ignore")
                            except Exception:
                                cmd = ""
                            # Håndtere basert på kommando
                            håndter_kommando(ser, cmd)
                            # Tømme buffer
                            line_buf.clear()
                        continue
                    
                    # Legge til tegn innenfor gitt lengde 
                    if len(line_buf) < MAX_CMD_LINE_LEN:
                        line_buf.append(b)
                    else:
                        # Dersom bufferen blir full, tømme den
                        print("[WARN] Kommando-buffer overflow -> reset")
                        line_buf.clear()

            # Prosessere data
            prosesser_planlagte_send(ser)
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\nAvslutter...")
        ser.close()

if __name__ == "__main__":
    main()
