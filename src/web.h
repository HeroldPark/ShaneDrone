// web.h - HTTP 서버 & WebSocket (대시보드) 모듈 API
#pragma once

namespace web {
  // HTTP 서버/웹소켓 시작 (라우팅, CORS, 대시보드 준비)
  void begin();

  // 매 프레임(루프)마다 호출: client 처리 + WebSocket loop + 주기적 브로드캐스트
  void loop();
}
