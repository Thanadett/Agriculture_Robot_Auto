#include "serial_read2.h"
#include <string.h>

namespace sr2 {

// ---------------- Internal storage ----------------
struct Route {
  const char*    prefix;
  LineHandlerCtx fn;
  void*          ctx;
};

static Route  g_routes[8];           // เพิ่มขนาดได้ตามต้องการ / enlarge if needed
static int    g_route_count = 0;
static String g_rx;                   // line buffer
static size_t g_max_len = 240;

// ---------------- Helpers ----------------
static inline bool startsWithCI(const String& s, const char* p) {
  size_t n = strlen(p);
  return s.length() >= (int)n && s.substring(0, n).equalsIgnoreCase(p);
}

// ---------------- Built-in adapters (thunks) ----------------
// Adapt target functions to LineHandlerCtx signature

static bool servo_thunk(const String& line, void* /*ctx*/) {
  // delegate to servo module
  return button_handle_line(line);
}

static bool stepper_thunk(const String& line, void* ctx) {
  auto* stp = reinterpret_cast<UnifiedStepper*>(ctx);
  if (!stp) return false;
  return stepper_handle_line(line, *stp);
}

// ---------------- Public APIs ----------------
void set_max_line_len(size_t n) { g_max_len = n; }

void init() {
  g_route_count = 0;
  g_rx = "";
}

bool add_route(const char* prefix, LineHandlerCtx fn, void* ctx) {
  if (!prefix || !fn) return false;
  if (g_route_count >= (int)(sizeof(g_routes)/sizeof(g_routes[0]))) return false;
  g_routes[g_route_count++] = Route{prefix, fn, ctx};
  return true;
}

void register_btn() {
  // "BTN A=DOWN B=UP X=DOWN" => servo
  (void) add_route("BTN", &servo_thunk, nullptr);
}

void register_stp(UnifiedStepper* stepper_ctx_ptr) {
  // "STP UP=DOWN DOWN=UP" => stepper
  (void) add_route("STP", &stepper_thunk, stepper_ctx_ptr);
}

void poll() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      if (g_rx.length() > 0) {
        bool handled = false;
        for (int i = 0; i < g_route_count; ++i) {
          if (startsWithCI(g_rx, g_routes[i].prefix)) {
            handled = g_routes[i].fn(g_rx, g_routes[i].ctx);
            break;
          }
        }
        if (!handled) {
          // No registered prefix matched
          Serial.printf("[sr2] Unhandled line: %s\n", g_rx.c_str());
        }
        g_rx = "";
      }
    } else {
      if ((int)g_rx.length() < (int)g_max_len) g_rx += c;
      // else: truncate overflow silently
    }
  }
}

} // namespace sr2
