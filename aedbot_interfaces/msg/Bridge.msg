# 목적지 도착 후 nav2에서 bridge로 보내는 메세지
bool nav2_to_bridge

# 목적지 도착 후 CPR, WEB에 보내는 메세지
bool arrive_destination

# CPR이 완료되었을 때 Navigation에 보내는 메세지
int32 complete_cpr

# WEB 통신이 끝났을 때 Navigation에 보내는 메세지
bool complete_web

# bridge에서 nav2로 보내는 메세지
bool bridge_to_nav2