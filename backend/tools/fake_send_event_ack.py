"""
fake_send_event_ack.py — 프론트엔드 "확인" 버튼 대역 (한 번 쏘고 끝나는 스크립트)

프론트엔드가 아직 완성되지 않아 이벤트 재개를 시험할 수 없을 때 쓴다.
서버가 아니다. 접속 -> 메시지 하나 전송 -> 종료. curl 과 같은 성격이다.

    관제자가 팝업의 "확인" 버튼을 눌렀을 때 프론트가 보내는 것과
    **똑같은 메시지 한 줄**을 대신 보낸다.

        {"event_type": "event_ack"}

프론트엔드가 완성되면 **이 스크립트를 실행하지 않으면 그만이다.**
백엔드 코드는 한 줄도 건드리지 않았으므로 지울 필요도 없다.

전체 경로 — 가짜는 맨 앞 한 칸뿐이고 나머지는 전부 실제 코드다
------------------------------------------------------------------
    fake_send_event_ack.py ──/ws/frontend──> 백엔드 ──/ws/jetson──>
        websocket_client._recv_loop ──/server/inbound──>
            event_gate_node ──/event/active=false──> 순찰 재개

실행 방법
---------
    cd backend
    venv/bin/python tools/fake_send_event_ack.py                 # 127.0.0.1:8000
    venv/bin/python tools/fake_send_event_ack.py --server ws://192.168.0.5:8000
    venv/bin/python tools/fake_send_event_ack.py --watch 3       # 응답도 잠깐 지켜보기

미리 떠 있어야 하는 것
----------------------
    1) 백엔드            venv/bin/uvicorn main:app --host 0.0.0.0 --port 8000
    2) websocket_client  젯슨에서, 또는 노트북에서 서버 주소를 바꿔서
       ros2 run ship_ugv_perception websocket_client \
           --ros-args -p server_ws_url:=ws://127.0.0.1:8000/ws/jetson
    3) event_gate_node   navigation.launch.py 가 patrol:=true 로 띄운다

젯슨이 접속해 있지 않으면 백엔드가
"⚠️ [중계] event_ack 전달 실패: 젯슨 미접속 상태" 를 출력한다.
그 경우 2)번이 떠 있는지 확인할 것.
"""

import argparse
import asyncio
import json
import sys

try:
    from websockets.asyncio.client import connect
except ImportError:                      # websockets 12 이하 호환
    try:
        from websockets.client import connect
    except ImportError:
        print('websockets 가 없다.  python3 -m pip install --user websockets')
        sys.exit(1)


async def send_ack(server: str, watch: float):
    url = server.rstrip('/') + '/ws/frontend'
    print(f'접속: {url}')
    try:
        async with connect(url) as ws:
            payload = json.dumps({'event_type': 'event_ack'})
            await ws.send(payload)
            print(f'전송: {payload}')
            print()
            print('  이제 아래가 순서대로 일어나야 한다:')
            print('    백엔드 로그       "[중계] event_ack -> 젯슨"')
            print('    websocket_client  "[수신] /server/inbound 로 중계"')
            print('    event_gate_node   "▶️ 재개 — 관제 확인 버튼"')
            print('    로봇              순찰 재개')

            if watch > 0:
                print()
                print(f'  서버에서 오는 메시지를 {watch:.0f}초간 지켜본다 (Ctrl+C 로 중단)')
                try:
                    while True:
                        msg = await asyncio.wait_for(ws.recv(), timeout=watch)
                        print(f'    <- {msg[:200]}')
                except asyncio.TimeoutError:
                    print('    (더 이상 수신 없음)')
    except OSError as e:
        print(f'❌ 접속 실패: {e}')
        print('   백엔드가 떠 있는지 확인:')
        print('     cd backend && venv/bin/uvicorn main:app --host 0.0.0.0 --port 8000')
        return 1
    return 0


def main():
    ap = argparse.ArgumentParser(
        description='프론트엔드 "확인" 버튼 대역 — event_ack 한 번 전송')
    ap.add_argument('--server', default='ws://127.0.0.1:8000',
                    help='백엔드 주소 (기본 ws://127.0.0.1:8000)')
    ap.add_argument('--watch', type=float, default=0.0, metavar='SEC',
                    help='전송 후 이 시간만큼 서버 수신 메시지를 출력')
    a = ap.parse_args()
    try:
        return asyncio.run(send_ack(a.server, a.watch))
    except KeyboardInterrupt:
        return 0


if __name__ == '__main__':
    sys.exit(main())
