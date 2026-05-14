#pragma once

// ── Core ──────────────────────────────────────────────────────────────────────
#include "TDristiCom/Types.hpp"
#include "TDristiCom/Logger.hpp"
#include "TDristiCom/Serializer.hpp"

// ── Transport ─────────────────────────────────────────────────────────────────
#include "TDristiCom/transport/ITransport.hpp"
#include "TDristiCom/transport/IStream.hpp"
#include "TDristiCom/transport/UnixTransport.hpp"
#include "TDristiCom/transport/TcpTransport.hpp"
#include "TDristiCom/transport/MmapStream.hpp"
#include "TDristiCom/transport/UdpStream.hpp"

// ── Session ───────────────────────────────────────────────────────────────────
#include "TDristiCom/session/SessionManager.hpp"

// ── Service ───────────────────────────────────────────────────────────────────
#include "TDristiCom/service/ServiceServer.hpp"
#include "TDristiCom/service/ServiceClient.hpp"

// ── Action ────────────────────────────────────────────────────────────────────
#include "TDristiCom/action/GoalHandle.hpp"
#include "TDristiCom/action/ActionServer.hpp"
#include "TDristiCom/action/ActionClient.hpp"
