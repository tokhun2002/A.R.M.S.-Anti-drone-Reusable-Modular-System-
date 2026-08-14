#!/usr/bin/env python3
"""
Ranger Micro USB-CRSF 테스트 + 텔레메트리 디코더 (Jetson/Linux)
==============================================================
설정:  /dev/ttyUSB0 · 115200 8N1 · DTR/RTS OFF · 0x16 RC @ 100Hz

수신 디코딩:
  0x14 LinkStatistics  : LQ / RSSI / SNR (RSSI는 signed 보정)
  0x08 Battery         : 전압 / 전류 / 사용 mAh / 잔량 %
  0x02 GPS             : 위도 / 경도 / 속도 / 방위 / 고도 / 위성수
  0x07 Vario           : 상승/하강속도
  0x1E Attitude        : pitch / roll / yaw
  0x21 FlightMode      : 비행모드 문자열
  그 외 타입은 "unknown 0xNN"로 표시

※ Battery/GPS/Attitude/Mode 는 Pixhawk가 CRSF 텔레메트리로 내보내야 들어옴.
  RP3만 켠 벤치 상태에선 LinkStats(0x14)/Sync(0x3A)만 보이는 게 정상.

의존성: pip3 install pyserial
실행:   python3 ranger_usb_test.py [/dev/ttyUSB0]
"""

import sys, time, glob, struct, serial

BAUD, RATE_HZ = 115200, 100
CRSF_SYNC, TYPE_RC = 0xC8, 0x16
CH_MIN, CH_MID = 172, 992
CHANNELS = [CH_MID, CH_MID, CH_MIN, CH_MID] + [CH_MIN] * 12   # R,P,T,Y + AUX

# 프레임 타입 이름
NAMES = {0x02:"GPS",0x07:"Vario",0x08:"Battery",0x09:"Baro",0x0B:"Heartbeat",
         0x14:"LinkStats",0x16:"RC",0x1E:"Attitude",0x21:"FlightMode",
         0x29:"DeviceInfo",0x3A:"RadioSync"}

# ---- CRC8 / DVB-S2 ------------------------------------------------------
def _crc8(c,b):
    c ^= b
    for _ in range(8):
        c = ((c<<1)^0xD5)&0xFF if (c&0x80) else (c<<1)&0xFF
    return c
def crc8(d):
    c=0
    for b in d: c=_crc8(c,b)
    return c

def s8(v):  return v-256 if v>=128 else v          # signed int8

# ---- RC 송신 프레임 -----------------------------------------------------
def pack_channels(ch):
    out=bytearray(); bits=n=0
    for c in ch:
        bits|=(int(c)&0x7FF)<<n; n+=11
        while n>=8: out.append(bits&0xFF); bits>>=8; n-=8
    if n: out.append(bits&0xFF)
    return bytes(out)
def build_rc_frame(ch):
    body=bytes([TYPE_RC])+pack_channels(ch)
    return bytes([CRSF_SYNC,len(body)+1])+body+bytes([crc8(body)])

# ---- 텔레메트리 디코더 --------------------------------------------------
def dec_battery(p):
    if len(p)<8: return None
    v=struct.unpack('>H',p[0:2])[0]/10.0        # V
    i=struct.unpack('>H',p[2:4])[0]/10.0        # A
    mah=(p[4]<<16)|(p[5]<<8)|p[6]               # mAh
    pct=p[7]                                     # %
    return {"v":v,"i":i,"mah":mah,"pct":pct}
def dec_gps(p):
    if len(p)<15: return None
    lat,lon,gs,hdg,alt=struct.unpack('>iiHHH',p[0:14]); sats=p[14]
    return {"lat":lat/1e7,"lon":lon/1e7,"kmh":gs/10.0,"hdg":hdg/100.0,
            "alt":alt-1000,"sats":sats}          # alt: m, offset 1000
def dec_attitude(p):
    if len(p)<6: return None
    pit,rol,yaw=struct.unpack('>hhh',p[0:6])
    r=57.2958/10000.0
    return {"pitch":pit*r,"roll":rol*r,"yaw":yaw*r}   # deg
def dec_vario(p):
    if len(p)<2: return None
    return {"vspd":struct.unpack('>h',p[0:2])[0]/100.0}   # m/s
def dec_mode(p):
    return {"mode":p.split(b'\x00')[0].decode('ascii','ignore')}
def dec_link(p):
    if len(p)<10: return None
    return {"uplq":p[2],"upsnr":s8(p[3]),"uprssi":s8(p[0]),
            "dnlq":p[8],"dnsnr":s8(p[9]),"dnrssi":s8(p[7])}

# ---- 수신 파서 ----------------------------------------------------------
class Parser:
    OK=(0xC8,0xEE,0xEA,0xEC)
    def __init__(self): self.buf=bytearray()
    def feed(self,data):
        self.buf.extend(data); out=[]
        while len(self.buf)>=2:
            if self.buf[0] not in self.OK: self.buf.pop(0); continue
            L=self.buf[1]
            if L<2 or L>62: self.buf.pop(0); continue
            tot=L+2
            if len(self.buf)<tot: break
            fr=bytes(self.buf[:tot]); body=fr[2:-1]
            if crc8(body)==fr[-1]:
                out.append((fr[2],body[1:])); del self.buf[:tot]
            else: self.buf.pop(0)
        return out

# ---- 포트 (DTR/RTS OFF) -------------------------------------------------
def open_port(port):
    s=serial.Serial()
    s.port=port; s.baudrate=BAUD
    s.bytesize=serial.EIGHTBITS; s.parity=serial.PARITY_NONE; s.stopbits=serial.STOPBITS_ONE
    s.timeout=0; s.rtscts=False; s.dsrdtr=False; s.dtr=False; s.rts=False
    s.open()
    try: s.dtr=False; s.rts=False
    except Exception: pass
    return s
def find_port():
    c=sorted(glob.glob("/dev/ttyUSB*")+glob.glob("/dev/ttyACM*"))
    return c[0] if c else None

# ---- 메인 --------------------------------------------------------------
def main():
    port=sys.argv[1] if len(sys.argv)>1 else find_port()
    if not port:
        print("❌ /dev/ttyUSB* 없음. 'ls -l /dev/ttyUSB*' 확인 후 인자로 넘겨."); sys.exit(1)
    ser=open_port(port); ps=Parser(); frame=build_rc_frame(CHANNELS)

    sent=rx_raw=rx_valid=0
    seen=set(); link=batt=gps=att=None; mode=None
    period=1.0/RATE_HZ; nt=time.monotonic(); nr=nt+1.0

    print(f"✅ {port} @ {BAUD} · RC 0x16 {RATE_HZ}Hz 송신. Ctrl-C 종료.\n")
    try:
        while True:
            ser.write(frame); sent+=1
            data=ser.read(512)
            if data:
                rx_raw+=len(data)
                for t,p in ps.feed(data):
                    rx_valid+=1; seen.add(t)
                    if   t==0x14: link=dec_link(p)
                    elif t==0x08: batt=dec_battery(p)
                    elif t==0x02: gps =dec_gps(p)
                    elif t==0x1E: att =dec_attitude(p)
                    elif t==0x21: mode=dec_mode(p).get("mode")
            now=time.monotonic()
            if now>=nr:
                nr+=1.0
                parts=[f"sent={sent} RXvalid={rx_valid}"]
                if link: parts.append(f"LQ {link['uplq']}/{link['dnlq']} SNR{link['upsnr']} RSSI{link['uprssi']}dBm")
                if batt: parts.append(f"🔋 {batt['v']:.2f}V {batt['i']:.1f}A {batt['mah']}mAh {batt['pct']}%")
                if gps:  parts.append(f"GPS {gps['lat']:.6f},{gps['lon']:.6f} {gps['kmh']:.1f}km/h alt{gps['alt']}m sat{gps['sats']}")
                if att:  parts.append(f"ATT r{att['roll']:.0f} p{att['pitch']:.0f} y{att['yaw']:.0f}")
                if mode: parts.append(f"MODE:{mode}")
                tag="🟢" if rx_valid>0 else "🔴"
                print(f"[{tag}] " + " | ".join(parts))
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        types=", ".join(f"0x{t:02X}({NAMES.get(t,'?')})" for t in sorted(seen)) or "없음"
        print(f"\n종료. sent={sent} RXraw={rx_raw}B RXvalid={rx_valid}")
        print(f"수신한 프레임 타입: {types}")
        if batt: print(f"마지막 배터리: {batt['v']:.2f}V {batt['pct']}%")

if __name__=="__main__":
    main()