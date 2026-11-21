#include "protocol_eaton.h"

#include "constants_hid.h"
#include "constants_ups.h"
#include "esphome/core/log.h"
#include "esphome/core/util.h"  // millis()

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace esphome {
namespace ups_hid {

static const char *const EATON_TAG = "ups_hid.eaton";

// HID-PDC reports on these devices fit in a single full-speed control transfer.
static constexpr size_t MAX_REPORT_BYTES = 128;

// Battery collection usages all live on the Power Device page (0x10..0x13), but
// a field on the Battery System page (0x85) is in a battery context by itself.
bool EatonHidpdProtocol::in_battery_collection(const Field &f) const {
  for (const auto &seg : f.path) {
    if (seg.page == HID_USAGE_PAGE_POWER_DEVICE &&
        (seg.usage == HID_USAGE_POW_BATTERY_SYSTEM || seg.usage == HID_USAGE_POW_BATTERY_SYSTEM_ID ||
         seg.usage == HID_USAGE_POW_BATTERY || seg.usage == HID_USAGE_POW_BATTERY_ID))
      return true;
  }
  return false;
}

// HID-PDC base-unit codes and the decimal exponent each one already carries
// (HID base units are g/cm/s/mA, so e.g. a volt is 10^7 of them). A device's
// Unit Exponent is relative to that, so the scale of a field is
// 10^(UnitExponent - canonical), exactly like NUT's HIDUnits table in
// drivers/libhid.c. Unknown unit codes keep the raw exponent.
struct UnitExpo {
  int32_t unit;
  int8_t expo;
};
static const UnitExpo kHidUnits[] = {
    {0x00000000, 0},  // none / percent
    {0x00F0D121, 7},  // volt
    {0x00100001, 0},  // ampere
    {0x0000D121, 7},  // VA
    {0x0000D121, 7},  // watt
    {0x00001001, 0},  // second
    {0x00010001, 0},  // kelvin
    {0x0000F001, 0},  // hertz
    {0x00101001, 0},  // ampere-second
};

static int unit_scale_expo(int32_t unit, int32_t unit_exp) {
  for (const auto &u : kHidUnits) {
    if (u.unit == unit)
      return static_cast<int>(unit_exp) - u.expo;
  }
  return static_cast<int>(unit_exp);
}

namespace {
// Two's complement of a descriptor item of `bits` width.
int32_t sign_extend(uint32_t v, uint8_t bits) {
  if (bits == 0 || bits >= 32)
    return static_cast<int32_t>(v);
  const uint32_t m = 1u << (bits - 1);
  return static_cast<int32_t>((v ^ m) - m);
}

bool isnan_(float v) { return std::isnan(v); }

float clamp_pct(float v) {
  if (v < 0.0f)
    return 0.0f;
  if (v > 100.0f)
    return 100.0f;
  return v;
}
}  // namespace

// ---------------------------------------------------------------- transport

bool EatonHidpdProtocol::detect() {
  if (!parent_->is_connected()) {
    return false;
  }
  return fetch_and_parse();
}

bool EatonHidpdProtocol::initialize() {
  if (fields_.empty() && !fetch_and_parse()) {
    return false;
  }
  read_strings();
  return true;
}

bool EatonHidpdProtocol::fetch_and_parse() {
  std::vector<uint8_t> rd;
  const esp_err_t err = parent_->get_report_descriptor(rd);
  if (err != ESP_OK || rd.size() < 8) {
    last_error_ = std::string("GET_DESCRIPTOR(0x22) failed: ") + esp_err_to_name(err) +
                  " (" + std::to_string(rd.size()) + " bytes read)";
    ESP_LOGW(EATON_TAG, "No HID report descriptor available (%s) - this device needs another protocol",
             esp_err_to_name(err));
    return false;
  }

  descriptor_bytes_ = rd.size();
  map_dump_pos_ = 0;
  map_header_logged_ = false;
  decoded_shown_ = 0;
  poll_cycles_ = 0;
  map_lines_logged_ = 0;
  report_strikes_.clear();
  dead_reports_.clear();
  if (!parse_descriptor(rd)) {
    last_error_ = "descriptor read OK (" + std::to_string(descriptor_bytes_) +
                  " bytes) but field map unusable: " + std::to_string(fields_.size()) + " fields found";
    ESP_LOGW(EATON_TAG, "HID-PDC field map unusable (descriptor %u bytes, %zu fields)",
             static_cast<unsigned>(descriptor_bytes_), fields_.size());
    fields_.clear();
    return false;
  }

  // Diagnostics for the component's deferred logging: the detection context is
  // inside the boot log flood, so these ESP_LOG lines below are usually lost;
  // the same content is queued here and re-logged on the first poll cycle.
  last_error_.clear();
  pending_logs_.clear();
  char line[240];
  int used = snprintf(line, sizeof(line),
                      "Eaton HID-PDC map: %zu fields across %zu input reports (descriptor %u bytes) bits:",
                      fields_.size(), input_reports_.size(), static_cast<unsigned>(descriptor_bytes_));
  for (const uint8_t rid : input_reports_) {
    if (used > 0 && used < static_cast<int>(sizeof(line)) - 14)
      used += snprintf(line + used, sizeof(line) - used, " %02X/%u", rid, static_cast<unsigned>(input_bits_[rid]));
  }
  pending_logs_.emplace_back(line);
  ESP_LOGI(EATON_TAG, "%s", line);
  const std::string topo = parent_->usb_topology_summary();
  if (!topo.empty()) {
    ESP_LOGI(EATON_TAG, "USB topology: %s", topo.c_str());
    pending_logs_.emplace_back("USB topology: " + topo);
  }
  // Stash only a small prefix of the map for the deferred re-logging (the
  // read_data() first-cycle dump streams the complete map in a context that
  // survives the boot flood anyway): the full 5PX descriptor has hundreds of
  // fields and queueing them all wastes internal RAM and poll cycles.
  size_t queued = 0;
  for (const auto &f : fields_) {
    if (f.usage == 0)
      continue;  // padding/constant
    snprintf(line, sizeof(line), "  rpt 0x%02X bit %2u size %2u  page 0x%02X usage 0x%02X  log %d..%d  unit 0x%04X exp %d  %s",
             f.report_id, static_cast<unsigned>(f.bit_offset), f.size, f.page, f.usage, static_cast<int>(f.log_min),
             static_cast<int>(f.log_max), static_cast<unsigned>(f.unit), static_cast<int>(f.unit_exp),
             path_to_string(f.path).c_str());
    if (queued++ < 10)
      pending_logs_.emplace_back(line);
    ESP_LOGD(EATON_TAG, "%s", line);
  }
  if (queued > 10) {
    snprintf(line, sizeof(line), "  ... %zu more fields (full map streamed during the first poll cycles)", queued - 10);
    pending_logs_.emplace_back(line);
  }
  return true;
}

bool EatonHidpdProtocol::read_strings() {
  if (!mfr_.empty() || !model_.empty())
    return true;
  // USB string indexes 1..3 (manufacturer / product / serial) - the generic
  // protocol leaves these empty, which is why NUT clients show "Generic".
  parent_->get_string_descriptor(1, mfr_);
  parent_->get_string_descriptor(2, model_);
  parent_->get_string_descriptor(3, serial_);
  if (!mfr_.empty() || !model_.empty()) {
    ESP_LOGI(EATON_TAG, "USB strings: manufacturer=\"%s\" product=\"%s\" serial=\"%s\"", mfr_.c_str(),
             model_.c_str(), serial_.c_str());
  }
  return !mfr_.empty() || !model_.empty();
}

// ------------------------------------------------------------ HID descriptor

bool EatonHidpdProtocol::parse_descriptor(const std::vector<uint8_t> &rd) {
  fields_.clear();
  input_reports_.clear();
  input_bits_.clear();

  uint16_t report_size = 0;
  uint16_t report_count = 0;
  uint8_t report_id = 0;
  uint16_t page = 0;
  int32_t log_min = 0;
  int32_t log_max = 0;
  uint32_t log_max_raw = 0;
  uint8_t log_max_bits = 0;
  int32_t unit = 0;
  int32_t unit_exp = 0;
  bool has_range = false;
  uint16_t range_min = 0;
  uint16_t range_max = 0;
  std::vector<uint16_t> local_usages;

  std::vector<PathSeg> path;
  std::map<uint8_t, uint32_t> bit_in;    // per report ID, independent bit cursors
  std::map<uint8_t, uint32_t> bit_feat;

  size_t i = 0;
  while (i < rd.size()) {
    const uint8_t b = rd[i];
    if (b == 0xFE) {  // long item: 0xFE, byte count, payload
      if (i + 1 >= rd.size())
        break;
      const size_t n = rd[i + 1];
      i += 2 + n;
      continue;
    }
    uint8_t sz = b & 0x03;
    if (sz == 3)
      sz = 4;
    if (i + 1 + sz > rd.size())
      break;  // truncated descriptor
    const uint8_t tag = b & 0xFC;  // 6-bit tag + 2-bit size
    uint32_t val = 0;
    for (uint8_t k = 0; k < sz; k++)
      val |= static_cast<uint32_t>(rd[i + 1 + k]) << (8 * k);
    i += 1 + sz;

    switch (tag) {
      case 0x04:  // Usage Page
        // Standard UPS descriptors use 1-byte pages (0x84 / 0x85). Some vendor
        // firmware emits a wide page item whose value also carries the usage in
        // the low byte (0x8404); normalise so the usage-page comparisons hold.
        if (val > 0xFF) {
          ESP_LOGD(EATON_TAG, "usage page 0x%04X normalised to 0x%02X", static_cast<unsigned>(val),
                   static_cast<unsigned>(val >> 8));
          page = static_cast<uint16_t>(val >> 8);
        } else {
          page = static_cast<uint16_t>(val);
        }
        break;
      case 0x08:  // Usage - part of a local usage list
        local_usages.push_back(static_cast<uint16_t>(val));
        break;
      case 0x18:  // Usage Minimum
        range_min = static_cast<uint16_t>(val);
        has_range = true;
        break;
      case 0x28:  // Usage Maximum
        range_max = static_cast<uint16_t>(val);
        has_range = true;
        break;
      case 0x14:  // Logical Minimum
        log_min = sign_extend(val, static_cast<uint8_t>(sz * 8));
        break;
      case 0x24:  // Logical Maximum
        log_max_raw = val;
        log_max = sign_extend(val, static_cast<uint8_t>(sz * 8));
        log_max_bits = static_cast<uint8_t>(sz * 8);
        // Vendors sometimes encode a full unsigned range in too few bytes, so
        // that it reads as a negative number (NUT does the same reinterpation).
        if (log_max < log_min)
          log_max = static_cast<int32_t>(val | (0xFFFFu << log_max_bits));
        break;
      case 0x54:  // Unit Exponent (log10 multiplier)
        unit_exp = sign_extend(val, static_cast<uint8_t>(sz * 8));
        break;
      case 0x64:  // Unit
        unit = static_cast<int32_t>(val);
        break;
      case 0x74:  // Report Size
        report_size = static_cast<uint16_t>(val);
        break;
      case 0x94:  // Report Count
        report_count = static_cast<uint16_t>(val);
        break;
      case 0x84:  // Report ID
        report_id = static_cast<uint8_t>(val);
        break;
      case 0xA0:  // Collection - the collection's usage is the pending Usage item
        if (!local_usages.empty())
          path.push_back({page, local_usages.back()});
        // Eaton/PowerDesign descriptors encode arrays of same-usage
        // collections as Collection(0x80|n); NUT tracks these as 0x00ff|n
        // path nodes so "Input.[1]", "Input.[2]", ... stay distinguishable.
        // Mirror that exactly.
        if (val >= 0x80)
          path.push_back({0x00FF, static_cast<uint16_t>(val & 0x7F)});
        local_usages.clear();
        has_range = false;
        break;
      case 0xC0:  // End Collection
        // An indexed collection pushed two nodes; End Collection pops both
        // (same rule as NUT's ITEM_END_COLLECTION handling).
        if (!path.empty()) {
          const PathSeg popped = path.back();
          path.pop_back();
          if (popped.page == 0x00FF && !path.empty())
            path.pop_back();
        }
        local_usages.clear();
        has_range = false;
        break;
      case 0x80:  // Input (0x81 = 0x80 | size 1)
      case 0xB0: {  // Feature (0xB1)
        std::map<uint8_t, uint32_t> &bits = (tag == 0x80) ? bit_in : bit_feat;
        uint32_t &off = bits[report_id];
        const uint16_t r_hi = std::max(range_min, range_max);
        for (uint16_t c = 0; c < report_count; c++) {
          uint16_t u = 0;
          if (has_range) {
            u = static_cast<uint16_t>(std::min<uint32_t>(range_min + c, r_hi));
          } else if (!local_usages.empty()) {
            // HID: if Report Count exceeds the usage list, the last usage repeats
            u = local_usages[c < local_usages.size() ? c : local_usages.size() - 1];
          }
          if (tag == 0x80 && fields_.size() < 1500) {
            // Hard cap: a malformed/vendored descriptor with huge Report
            // Counts must not exhaust the heap.
            Field f;
            f.report_id = report_id;
            f.bit_offset = off;
            f.size = report_size;
            f.page = page;
            // An array of constants carries no usage; fields with usage 0 are
            // still needed so the bit offsets of the following fields line up.
            f.usage = u;
            f.log_min = log_min;
            f.log_max = log_max;
            f.log_max_raw = log_max_raw;
            f.log_max_bits = log_max_bits;
            f.unit = unit;
            f.unit_exp = unit_exp;
            f.path = path;
            fields_.push_back(f);
          }
          off += report_size;
        }
        // Local items are consumed by the Main item that follows them
        local_usages.clear();
        has_range = false;
        break;
      }
      default:
        break;  // Physical Min/Max, String Index, Set Report, Local items, ...
    }
  }

  for (const auto &kv : bit_in) {
    input_reports_.push_back(kv.first);
    input_bits_[kv.first] = kv.second;
  }

  // Require a handful of recognisable HID-PDC usages before claiming the device.
  size_t usable = 0;
  for (const auto &f : fields_) {
    if (f.usage == 0)
      continue;
    if (f.page == HID_USAGE_PAGE_POWER_DEVICE || f.page == HID_USAGE_PAGE_BATTERY_SYSTEM)
      usable++;
  }
  return usable >= 3;
}

// ------------------------------------------------------------------ decoding

uint32_t EatonHidpdProtocol::extract_bits(const uint8_t *buf, size_t len, uint32_t bit_off, uint16_t bits) {
  uint32_t v = 0;
  for (uint16_t i = 0; i < bits && i < 32; i++) {
    const uint32_t b = bit_off + i;
    if ((b >> 3) >= len)
      break;
    if (buf[b >> 3] & (1u << (b & 7)))
      v |= (1u << i);
  }
  return v;
}

bool EatonHidpdProtocol::in_path(const Field &f, uint16_t page, uint16_t usage) const {
  for (const auto &seg : f.path) {
    if (seg.page == page && seg.usage == usage)
      return true;
  }
  return false;
}

// NUT-style disambiguation of indexed collections: returns the "[n]" instance
// number following a collection segment, or -1 when the collection carries no
// index node (single-instance devices).
int EatonHidpdProtocol::collection_index(const Field &f, uint16_t page, uint16_t usage) const {
  for (size_t k = 0; k + 1 < f.path.size(); k++) {
    if (f.path[k].page == page && f.path[k].usage == usage && f.path[k + 1].page == 0x00FF)
      return f.path[k + 1].usage;
  }
  return -1;
}

bool EatonHidpdProtocol::eaton_offset_candidate(const Field &f) const {
  if (f.size != 8 && f.size != 16 && f.size != 32)
    return false;
  const uint16_t vid = parent_ ? parent_->get_vendor_id() : 0;
  if (vid != 0x0463 && vid != 0x06DA)
    return false;
  if (f.log_min != 0)
    return false;  // already declared signed by the descriptor
  const uint32_t umax = (f.size >= 32) ? 0xFFFFFFFFu : ((1u << f.size) - 1u);
  // A "0 .. full-unsigned-range" measurement is Eaton's offset-binary encoding
  // (0x8000 == 0), exactly what NUT's eaton subdriver compensates for. Some
  // firmware describes that range with a narrow item (LogMax 0xFF in one byte),
  // so also accept a full item-width maximum.
  if (f.log_max_raw == umax)
    return true;
  return f.log_max_bits != 0 && f.log_max_bits != f.size &&
         f.log_max_raw == ((1u << f.log_max_bits) - 1u);
}

float EatonHidpdProtocol::scaled_value(const Field &f, uint32_t raw, bool measurement) const {
  int64_t v;
  if (f.log_min < 0) {
    v = sign_extend(raw, static_cast<uint8_t>(f.size));
  } else if (measurement && eaton_offset_candidate(f)) {
    v = static_cast<int64_t>(raw) - (1LL << (f.size - 1));
    ESP_LOGV(EATON_TAG, "  offset-binary: raw 0x%04X -> %lld", raw, static_cast<long long>(v));
  } else {
    v = static_cast<int64_t>(raw);
  }
  float out = static_cast<float>(v);
  const int expo = unit_scale_expo(f.unit, f.unit_exp);
  if (expo != 0)
    out *= powf(10.0f, static_cast<float>(expo));
  return out;
}

void EatonHidpdProtocol::collect(const Field &f, uint32_t raw, Decoded &d) const {
  if (f.usage == 0)
    return;

  const bool battery_page = (f.page == HID_USAGE_PAGE_BATTERY_SYSTEM);
  const bool power_page = (f.page == HID_USAGE_PAGE_POWER_DEVICE);
  if (!battery_page && !power_page)
    return;

  const bool battery_ctx = battery_page || in_battery_collection(f);
  // NUT's Eaton (mge-hid) subdriver reads most 5P/5PX/5SC values from the
  // PowerSummary collection rather than Battery/Input/Output collections.
  const bool summary_ctx = in_path(f, HID_USAGE_PAGE_POWER_DEVICE, HID_USAGE_POW_POWER_SUMMARY);

  // --- status bits: 1-bit fields ------------------------------------------
  if (f.size == 1) {
    const bool set = (raw != 0);
    d.has_status = true;
    if (battery_page) {
      switch (f.usage) {
        case HID_USAGE_BAT_CHARGING:
          d.charging = d.charging || set;
          break;
        case HID_USAGE_BAT_DISCHARGING:
          d.on_battery = d.on_battery || set;
          d.discharging = d.discharging || set;
          break;
        case HID_USAGE_BAT_FULLY_CHARGED:
          d.fully_charged = d.fully_charged || set;
          break;
        case HID_USAGE_BAT_NEED_REPLACEMENT:
          d.need_replacement = d.need_replacement || set;
          break;
        case HID_USAGE_BAT_AC_PRESENT:
          if (set)
            d.online = true;
          break;
        case HID_USAGE_BAT_POWER_FAIL:
          d.on_battery = d.on_battery || set;
          break;
        case HID_USAGE_BAT_BATTERY_PRESENT:
          break;  // informational; a missing battery shows up as no battery data
        case HID_USAGE_BAT_VOLTAGE_OUT_OF_RANGE:
          d.voltage_out_of_range = d.voltage_out_of_range || set;
          break;
        default:
          break;
      }
      return;
    }
    switch (f.usage) {
      case HID_USAGE_POW_VOLTAGE:  // a 1-bit Voltage item = "this input/output is present"
        if (set)
          d.online = true;
        break;
      case HID_USAGE_POW_USED:
        // "PowerConverter.Input.[1].PresentStatus.Used" => utility converter
        // feeding the load (online); "Input.[3]...Used" => energy comes from
        // the battery-side converter (on battery) — NUT mge-hid semantics.
        if (in_path(f, HID_USAGE_PAGE_POWER_DEVICE, HID_USAGE_POW_INPUT)) {
          const int idx = collection_index(f, HID_USAGE_PAGE_POWER_DEVICE, HID_USAGE_POW_INPUT);
          if (idx == 3)
            d.on_battery = d.on_battery || set;
          else
            d.online = d.online || set;
        } else if (set && summary_ctx) {
          d.online = true;
        }
        break;
      case HID_USAGE_POW_GOOD:
        if (set)
          d.online = true;
        break;
      case HID_USAGE_POW_INTERNAL_FAILURE:
        d.internal_failure = d.internal_failure || set;
        break;
      case HID_USAGE_POW_VOLTAGE_OUT_OF_RANGE:
        d.voltage_out_of_range = d.voltage_out_of_range || set;
        break;
      case HID_USAGE_POW_OVERLOAD:
        d.overload = d.overload || set;
        break;
      case HID_USAGE_POW_SHUTDOWN_IMMINENT:
        d.shutdown_imminent = d.shutdown_imminent || set;
        break;
      default:
        break;
    }
    return;
  }

  // --- numeric fields -----------------------------------------------------
  const bool measurement = power_page && (f.usage == HID_USAGE_POW_VOLTAGE || f.usage == HID_USAGE_POW_FREQUENCY ||
                                          f.usage == HID_USAGE_POW_CURRENT || f.usage == HID_USAGE_POW_ACTIVE_POWER ||
                                          f.usage == HID_USAGE_POW_APPARENT_POWER);
  const float v = scaled_value(f, raw, measurement);

  if (power_page) {
    switch (f.usage) {
      case HID_USAGE_POW_VOLTAGE:
        if (battery_ctx) {
          d.voltage_battery = v;
        } else if (summary_ctx) {
          // NUT mge-hid: PowerSummary.Voltage is the battery voltage on the
          // Eaton 5P/5PX/5SC series (mge_battery_voltage_fun).
          d.voltage_battery = v;
        } else if (in_path(f, HID_USAGE_PAGE_POWER_DEVICE, HID_USAGE_POW_INPUT)) {
          // First Input collection wins == "Input.[1]" in NUT's paths; the
          // bypass/secondary inputs ([2]/[3]) carry near-identical values.
          if (isnan_(d.voltage_input))
            d.voltage_input = v;
        } else if (in_path(f, HID_USAGE_PAGE_POWER_DEVICE, HID_USAGE_POW_OUTPUT)) {
          d.voltage_output = v;
        } else if (v < 60.0f) {
          d.voltage_battery = v;
        } else if (isnan_(d.voltage_input)) {
          d.voltage_input = v;
        } else {
          d.voltage_output = v;
        }
        break;
      case HID_USAGE_POW_CONFIG_VOLTAGE:
        if (battery_ctx || summary_ctx) {
          // PowerSummary.ConfigVoltage = nominal battery voltage on 5P-series
          // (mge_battery_voltage_nominal_fun in NUT).
          d.voltage_nom_battery = v;
        } else if (in_path(f, HID_USAGE_PAGE_POWER_DEVICE, HID_USAGE_POW_OUTPUT)) {
          d.voltage_nom_output = v;
        } else {
          d.voltage_nom_input = v;
        }
        break;
      case HID_USAGE_POW_FREQUENCY:
        d.frequency = v;
        break;
      case HID_USAGE_POW_PERCENT_LOAD:
        // Encoded as a 0..1 fraction when the device declares Unit Exponent -2.
        d.load_percent = (f.unit == 0 && f.unit_exp == -2) ? v * 100.0f : v;
        break;
      case HID_USAGE_POW_CURRENT:
        d.charge_current = v;
        break;
      case HID_USAGE_POW_CONFIG_ACTIVE_POWER:
        d.realpower_nominal = v;
        break;
      case HID_USAGE_POW_CONFIG_APPARENT_POWER:
        d.apparent_power_nominal = v;
        break;
      case HID_USAGE_POW_LOW_VOLTAGE_TRANSFER:
        d.transfer_low = v;
        break;
      case HID_USAGE_POW_HIGH_VOLTAGE_TRANSFER:
        d.transfer_high = v;
        break;
      default:
        break;
    }
    return;
  }

  // Battery System page. Percent is encoded unit-less with Unit Exponent -2
  // (i.e. a 0..1 fraction); some devices instead report 0..100 with exponent 0.
  switch (f.usage) {
    case HID_USAGE_BAT_RELATIVE_STATE_OF_CHARGE:
      d.battery_level = (f.unit == 0 && f.unit_exp == -2) ? v * 100.0f : v;
      break;
    case HID_USAGE_BAT_REMAINING_CAPACITY:
      // NUT mge-hid maps battery.charge straight to PowerSummary.RemainingCapacity.
      // Percent fields declare unit 0 (none/percent), so the descriptor-scaled
      // value is already a percentage; a field with a real capacity unit (As)
      // is kept as raw capacity instead.
      if (f.unit == 0) {
        d.battery_level = v;
      } else {
        d.remaining_capacity = v;
      }
      break;
    case HID_USAGE_BAT_FULL_CHARGE_CAPACITY:
      if (f.unit == 0 && f.unit_exp == -2) {
        d.full_charge_capacity = v * 100.0f;
      } else {
        d.full_charge_capacity = v;
      }
      break;
    case HID_USAGE_BAT_RUN_TIME_TO_EMPTY:
      d.runtime_seconds = v;
      break;
    case HID_USAGE_BAT_AVERAGE_TIME_TO_EMPTY:
      d.runtime_avg_seconds = v;
      break;
    case HID_USAGE_BAT_AT_RATE_TIME_TO_EMPTY:
      if (isnan_(d.runtime_seconds))
        d.runtime_seconds = v;
      break;
    case HID_USAGE_BAT_WARNING_CAPACITY_LIMIT:
      if (f.unit == 0 && f.unit_exp == -2) {
        d.charge_warning = v * 100.0f;
      } else if (f.unit == 0 && f.log_max <= 100) {
        d.charge_warning = v;
      }
      break;
    default:
      break;
  }
}

void EatonHidpdProtocol::publish(const Decoded &d, UpsData &data) const {
  PowerData &p = data.power;
  BatteryData &b = data.battery;

  if (!isnan_(d.voltage_input))
    p.input_voltage = d.voltage_input;
  if (!isnan_(d.voltage_output))
    p.output_voltage = d.voltage_output;
  if (!isnan_(d.voltage_nom_input))
    p.input_voltage_nominal = d.voltage_nom_input;
  if (!isnan_(d.voltage_nom_output))
    p.output_voltage_nominal = d.voltage_nom_output;
  if (!isnan_(d.transfer_low))
    p.input_transfer_low = d.transfer_low;
  if (!isnan_(d.transfer_high))
    p.input_transfer_high = d.transfer_high;
  if (!isnan_(d.frequency))
    p.frequency = d.frequency;
  if (!isnan_(d.load_percent))
    p.load_percent = clamp_pct(d.load_percent);
  if (!isnan_(d.realpower_nominal))
    p.realpower_nominal = d.realpower_nominal;
  if (!isnan_(d.apparent_power_nominal))
    p.apparent_power_nominal = d.apparent_power_nominal;

  float level = d.battery_level;
  if (isnan_(level) && !isnan_(d.remaining_capacity) && !isnan_(d.full_charge_capacity) &&
      d.full_charge_capacity > 0.0f) {
    level = d.remaining_capacity / d.full_charge_capacity * 100.0f;
  }
  if (!isnan_(level))
    b.level = clamp_pct(level);
  if (!isnan_(d.voltage_battery))
    b.voltage = d.voltage_battery;
  if (!isnan_(d.voltage_nom_battery))
    b.voltage_nominal = d.voltage_nom_battery;
  if (!isnan_(d.charge_warning))
    b.charge_warning = d.charge_warning;

  float secs = isnan_(d.runtime_seconds) ? d.runtime_avg_seconds : d.runtime_seconds;
  if (!isnan_(secs) && secs > 0.0f)
    b.runtime_minutes = secs / 60.0f;

  // --- status text -------------------------------------------------------
  // "Online" is what the component's is_online() derives from a valid input
  // voltage, so prefer the decoded status bits only to label the state.
  const bool on_battery = d.on_battery || (d.has_status && !d.online);
  if (!mfr_.empty())
    data.device.manufacturer = mfr_;
  if (!model_.empty())
    data.device.model = model_;
  if (!serial_.empty())
    data.device.serial_number = serial_;

  if (on_battery) {
    p.status = status::ON_BATTERY;
    b.status = battery_status::DISCHARGING;
  } else if (d.online) {
    p.status = status::ONLINE;
    if (d.fully_charged)
      b.status = battery_status::FULLY_CHARGED;
    else if (d.charging)
      b.status = battery_status::CHARGING;
    else if (d.has_status)
      b.status = battery_status::NORMAL;
  }
  if (d.overload)
    p.status = p.status.empty() ? std::string("Online - Overload") : p.status + " - Overload";
  if (d.internal_failure)
    b.status = std::string(battery_status::FAULT) + battery_status::FAULT_SUFFIX;
  if (d.need_replacement) {
    if (b.status.empty())
      b.status = battery_status::NORMAL;
    if (b.status.find(battery_status::REPLACE_BATTERY_SUFFIX) == std::string::npos)
      b.status += battery_status::REPLACE_BATTERY_SUFFIX;
  }
  if (d.shutdown_imminent && b.status.find(battery_status::SHUTDOWN_IMMINENT_SUFFIX) == std::string::npos) {
    if (b.status.empty())
      b.status = battery_status::CRITICAL;
    b.status += battery_status::SHUTDOWN_IMMINENT_SUFFIX;
  }
}

// -------------------------------------------------------------------- reads

bool EatonHidpdProtocol::read_data(UpsData &data) {
  const uint32_t cycle_start = millis();
  if (fields_.empty() && !fetch_and_parse())
    return false;
  poll_cycles_++;

  Decoded d;
  bool decoded_any = false;

  // Time budget: the loop task blocks through this whole function and the
  // ESP-IDF task watchdog fires after ~5 s. Hard-stop USB work well below
  // that; whatever is left unread is picked up next cycle (cached reports
  // persist).
  constexpr uint32_t CYCLE_USB_BUDGET_MS = 2000;
  auto budget_left = [cycle_start]() -> uint32_t {
    const uint32_t used = millis() - cycle_start;
    return used >= CYCLE_USB_BUDGET_MS ? 0 : CYCLE_USB_BUDGET_MS - used;
  };

  // NUT reads the interrupt-IN endpoint: devices like the Eaton 5PX stream
  // their reports there spontaneously and may STALL or ignore GET_REPORT for
  // some report IDs. Drain a few interrupt reports into the per-report cache.
  if (!interrupt_unsupported_) {
    uint8_t ibuf[MAX_REPORT_BYTES];
    for (int i = 0; i < 2 && budget_left() > 500; i++) {
      size_t ilen = sizeof(ibuf);
      const esp_err_t ir = parent_->read_interrupt(ibuf, &ilen, 100);
      if (ir == ESP_ERR_NOT_SUPPORTED || ir == ESP_ERR_INVALID_STATE) {
        interrupt_unsupported_ = true;
        break;
      }
      if (ir != ESP_OK || ilen == 0)
        break;
      uint8_t rid = 0;
      const uint8_t *body = ibuf;
      size_t blen = ilen;
      if (!input_reports_.empty()) {  // report-ID devices prefix on the interrupt endpoint
        rid = ibuf[0];
        body = ibuf + 1;
        blen = ilen - 1;
      }
      if (blen == 0)
        continue;
      if (interrupt_seen_.find(rid) == interrupt_seen_.end()) {
        interrupt_seen_.insert(rid);
        ESP_LOGI(EATON_TAG, "interrupt report 0x%02X: %zu bytes", rid, blen);
      }
      report_cache_[rid].assign(body, body + blen);
    }
  }

  // GET_REPORT sweep. Reports the device refuses or hangs on are disabled
  // after two strikes — a hanging GET_REPORT otherwise leaks one transfer
  // (the crash-safe give-up path) every cycle forever.
  for (uint8_t report_id : input_reports_) {
    if (dead_reports_.find(report_id) != dead_reports_.end())
      continue;
    const uint32_t left = budget_left();
    if (left < 400) {
      if (poll_cycles_ <= 8)
        ESP_LOGW(EATON_TAG, "report sweep stopped at cycle budget (%u ms); rest deferred",
                 static_cast<unsigned>(CYCLE_USB_BUDGET_MS));
      else
        ESP_LOGD(EATON_TAG, "report sweep stopped at cycle budget; rest deferred");
      break;
    }
    uint8_t buf[MAX_REPORT_BYTES];
    size_t len = sizeof(buf);
    const esp_err_t err =
        parent_->hid_get_report(HID_REPORT_TYPE_INPUT, report_id, buf, &len, std::min<uint32_t>(700, left));
    if (err != ESP_OK || len == 0) {
      if (++report_strikes_[report_id] >= 2) {
        dead_reports_.insert(report_id);
        ESP_LOGW(EATON_TAG, "GET_REPORT input 0x%02X failed twice; using interrupt-endpoint data only for it",
                 report_id);
      } else {
        ESP_LOGD(EATON_TAG, "GET_REPORT input 0x%02X failed: %s", report_id, esp_err_to_name(err));
      }
      continue;
    }
    if (poll_cycles_ >= 2 && poll_cycles_ <= 4)
      ESP_LOGD(EATON_TAG, "input report 0x%02X [%zu bytes]: %s", report_id, len, hex_dump(buf, len).c_str());
    else
      ESP_LOGV(EATON_TAG, "input report 0x%02X [%zu bytes]: %s", report_id, len, hex_dump(buf, len).c_str());

    const uint8_t *body = buf;
    size_t body_len = len;
    if (report_id != 0 && body_len > 1 && body[0] == report_id) {
      body++;
      body_len--;  // control-transfer GET_REPORT echoes the report ID
    }
    report_cache_[report_id].assign(body, body + body_len);
  }

  // Decode from the cache: every report ID uses its freshest bytes, no
  // matter which source (interrupt or GET_REPORT) delivered them.
  for (uint8_t report_id : input_reports_) {
    auto it = report_cache_.find(report_id);
    if (it == report_cache_.end())
      continue;
    const std::vector<uint8_t> &cbuf = it->second;
    const uint8_t *body = cbuf.data();
    const size_t body_len = cbuf.size();

    const uint32_t described_bits = input_bits_[report_id];
    if (described_bits > body_len * 8 && poll_cycles_ <= 6) {
      ESP_LOGW(EATON_TAG, "input report 0x%02X is %zu bytes but the descriptor describes %u bits", report_id,
               body_len, static_cast<unsigned>(described_bits));
    }

    for (const auto &f : fields_) {
      if (f.report_id != report_id)
        continue;
      if (f.bit_offset + f.size > body_len * 8)
        continue;  // beyond what the device actually sent
      const uint32_t raw = extract_bits(body, body_len, f.bit_offset, f.size);
      collect(f, raw, d);
      if (f.usage != 0)
        decoded_any = true;
      if (f.usage != 0 && f.size > 1) {
        ESP_LOGV(EATON_TAG, "  page 0x%02X usage 0x%02X raw 0x%04X -> %.2f", f.page, f.usage, raw,
                 scaled_value(f, raw, false));
      }
    }
  }

  // First cycles: print the full picture here (this context demonstrably
  // survives logging, unlike the boot detection window). The dump is spread
  // across poll cycles a few lines at a time: hundreds of lines in one burst
  // overflow the logger task buffer and everything gets dropped. Cycle 1 is
  // skipped entirely — it collides with the boot-time config dump flood.
  if (first_cycle_ && poll_cycles_ >= 2) {
    if (!map_header_logged_) {
      map_header_logged_ = true;
      ESP_LOGD(EATON_TAG, "FIELD MAP (%zu fields / %zu input reports, descriptor %u bytes):", fields_.size(),
               input_reports_.size(), static_cast<unsigned>(descriptor_bytes_));
    }
    size_t emitted = 0;
    for (; map_dump_pos_ < fields_.size(); map_dump_pos_++) {
      const Field &f = fields_[map_dump_pos_];
      if (f.usage == 0)
        continue;
      ESP_LOGD(EATON_TAG, "  rpt 0x%02X bit %2u size %2u  page 0x%02X usage 0x%02X  log %d..%d  unit 0x%04X exp %d  %s",
               f.report_id, static_cast<unsigned>(f.bit_offset), f.size, f.page, f.usage, static_cast<int>(f.log_min),
               static_cast<int>(f.log_max), static_cast<unsigned>(f.unit), static_cast<int>(f.unit_exp),
               path_to_string(f.path).c_str());
      if (++map_lines_logged_ >= 140) {
        ESP_LOGI(EATON_TAG, "FIELD MAP log capped at %zu lines (%zu fields total; the rest stay in memory)",
                 map_lines_logged_, fields_.size());
        map_dump_pos_ = fields_.size();
        break;
      }
      if (++emitted >= 8) {
        map_dump_pos_++;  // for-clause increment is skipped by break
        break;
      }
    }
    if (map_dump_pos_ >= fields_.size())
      first_cycle_ = false;
  }

  // Repeat the value summary for its first cycles: if one cycle's log burst
  // gets dropped by the logger buffer, the summary still reaches the log.
  if (decoded_shown_ < 3 && poll_cycles_ >= 2) {
    decoded_shown_++;
    ESP_LOGD(EATON_TAG,
             "decoded: Vin=%.1f Vout=%.1f Vbat=%.1f nomBat=%.1f SOC=%.0f load=%.0f f=%.1f run=%.0fs | "
             "online=%d onbat=%d chg=%d full=%d ovld=%d fail=%d volimit=%d",
             d.voltage_input, d.voltage_output, d.voltage_battery, d.voltage_nom_battery, d.battery_level,
             d.load_percent, d.frequency, d.runtime_seconds, static_cast<int>(d.online),
             static_cast<int>(d.on_battery), static_cast<int>(d.charging), static_cast<int>(d.fully_charged),
             static_cast<int>(d.overload), static_cast<int>(d.internal_failure),
             static_cast<int>(d.voltage_out_of_range));
  }

  if (!decoded_any)
    return false;

  publish(d, data);
  return true;
}

// ------------------------------------------------------------------- helpers

std::string EatonHidpdProtocol::path_to_string(const std::vector<PathSeg> &path) {
  std::string s;
  for (const auto &seg : path) {
    char buf[16];
    snprintf(buf, sizeof(buf), "%s/%02X:%02X", s.empty() ? "" : " ", seg.page, seg.usage);
    s += buf;
  }
  return s.empty() ? std::string("<root>") : s;
}

std::string EatonHidpdProtocol::hex_dump(const uint8_t *buf, size_t len) {
  std::string s;
  char b[4];
  for (size_t i = 0; i < len; i++) {
    snprintf(b, sizeof(b), "%02X", buf[i]);
    s += b;
    if (i + 1 < len)
      s += " ";
  }
  return s;
}

// Creator used by UpsHidComponent::register_builtin_protocols().
std::unique_ptr<UpsProtocolBase> create_eaton_protocol(UpsHidComponent *parent) {
  return std::make_unique<EatonHidpdProtocol>(parent);
}

}  // namespace ups_hid
}  // namespace esphome
