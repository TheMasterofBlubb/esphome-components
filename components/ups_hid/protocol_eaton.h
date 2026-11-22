#pragma once

#include "ups_hid.h"
#include "data_composite.h"

#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace esphome {
namespace ups_hid {

/**
 * Descriptor-driven HID Power Device protocol.
 *
 * The generic fallback protocol decodes with hardcoded report IDs plus "find a
 * plausible byte" heuristics, which produces nonsense on devices whose field
 * positions and scalings are defined by the HID report descriptor - Eaton /
 * Metrics (VID 0x0463 / 0x06DA, e.g. the 5PX Gen2) being the prominent
 * example. This protocol instead:
 *
 *  1. reads the HID report descriptor (GET_DESCRIPTOR, type 0x22),
 *  2. builds a map of "usage + collection path -> report ID / bit offset /
 *     size / logical range / Unit / Unit Exponent",
 *  3. reads each input report and decodes every field by usage, scaling with
 *     the descriptor's Unit Exponent,
 *  4. applies Eaton's offset-binary encoding (0x8000 == 0 for 16-bit) for
 *     Eaton VIDs, matching NUT's HID_ITEM_FLAG_EATON_OFFSET.
 *
 * Everything it decodes is logged, so a VERBOSE log dump of a device shows the
 * full field map plus raw report bytes.
 */
class EatonHidpdProtocol : public UpsProtocolBase {
 public:
  explicit EatonHidpdProtocol(UpsHidComponent *parent) : UpsProtocolBase(parent) {}

  bool detect() override;
  bool initialize() override;
  bool read_data(UpsData &data) override;

  DeviceInfo::DetectedProtocol get_protocol_type() const override {
    return DeviceInfo::PROTOCOL_EATON_HIDPD;
  }
  std::string get_protocol_name() const override { return "Eaton HID PDC"; }

  std::string get_last_error() const override { return last_error_; }
  std::vector<std::string> take_pending_logs() override { return std::move(pending_logs_); }

 private:
  // A single collection level, e.g. {0x84, 0x1A} == "Power Device / Input".
  struct PathSeg {
    uint16_t page;
    uint16_t usage;
  };

  struct Field {
    uint8_t report_id{0};
    uint32_t bit_offset{0};  // from the first byte after the report ID
    uint16_t size{0};        // bits
    uint16_t page{0};        // usage page
    uint16_t usage{0};       // usage ID, 0 = constant/padding
    bool is_feature{false};  // FEATURE item (Eaton 5PX keeps its
                             // measurements — voltages, status — in feature
                             // reports, exactly what NUT libhid reads via
                             // ITEM_FEATURE); otherwise an INPUT item.
    int32_t log_min{0};      // Logical Minimum, signed interpretation
    int32_t log_max{0};      // Logical Maximum, signed interpretation
    uint32_t log_max_raw{0};  // Logical Maximum as read from the descriptor
    uint8_t log_max_bits{0};  // item size the Logical Maximum was encoded in
    int32_t unit{0};
    int32_t unit_exp{0};  // Unit Exponent (log10 multiplier), signed
    std::vector<PathSeg> path;
  };

  // Values decoded from one polling cycle before they are mapped onto UpsData.
  struct Decoded {
    float voltage_input = NAN, voltage_output = NAN, voltage_battery = NAN;
    float voltage_nom_input = NAN, voltage_nom_output = NAN, voltage_nom_battery = NAN;
    float transfer_low = NAN, transfer_high = NAN;
    float frequency = NAN;
    float load_percent = NAN;
    float realpower_nominal = NAN, apparent_power_nominal = NAN;
    float battery_level = NAN, remaining_capacity = NAN, full_charge_capacity = NAN;
    float charge_warning = NAN;
    float runtime_seconds = NAN, runtime_avg_seconds = NAN;
    float charge_current = NAN;
    bool has_status{false};
    bool online{false}, on_battery{false}, charging{false}, discharging{false};
    bool fully_charged{false}, need_replacement{false};
    bool overload{false}, internal_failure{false}, voltage_out_of_range{false};
    bool shutdown_imminent{false};
  };

  bool fetch_and_parse();
  bool parse_descriptor(const std::vector<uint8_t> &rd);
  bool read_strings();

  bool in_path(const Field &f, uint16_t page, uint16_t usage) const;
  int collection_index(const Field &f, uint16_t page, uint16_t usage) const;
  bool in_battery_collection(const Field &f) const;
  bool eaton_offset_candidate(const Field &f) const;
  float scaled_value(const Field &f, uint32_t raw, bool measurement) const;
  void collect(const Field &f, uint32_t raw, Decoded &d) const;
  void publish(const Decoded &d, UpsData &data) const;

  static uint32_t extract_bits(const uint8_t *buf, size_t len, uint32_t bit_off, uint16_t bits);
  static std::string path_to_string(const std::vector<PathSeg> &path);
  static std::string hex_dump(const uint8_t *buf, size_t len);

  std::vector<Field> fields_;
  std::vector<uint8_t> input_reports_;
  std::map<uint8_t, uint32_t> input_bits_;  // report ID -> payload bits described
  std::vector<uint8_t> feature_reports_;
  std::map<uint8_t, uint32_t> feature_bits_;
  uint32_t descriptor_bytes_{0};

  // Cached USB string descriptors (read once in initialize()).
  std::string mfr_, model_, serial_;

  // Diagnostics for the component's deferred logging (see UpsProtocolBase).
  std::string last_error_;
  std::vector<std::string> pending_logs_;  // [0] = summary, rest = field map
  // Cached diagnostics: read_data() runs in the normal poll window (unlike
  // detection, which sits inside the boot config-dump flood), so the first
  // cycles dump the field map, raw report bytes and decoded values at DEBUG.
  // Spread across cycles (20 map lines per poll) so the logger task buffer
  // cannot overflow and drop the whole burst.
  bool first_cycle_{true};
  bool map_header_logged_{false};
  size_t map_dump_pos_{0};
  int decoded_shown_{0};  // print the decoded summary on the first cycles
  int poll_cycles_{0};    // poll cycles since the map was (re)fetched

  // Latest bytes per report ID, fed by BOTH the interrupt endpoint (NUT
  // style, spontaneous) and GET_REPORT (control poll).
  std::map<uint8_t, std::vector<uint8_t>> report_cache_;
  std::map<uint8_t, std::vector<uint8_t>> feature_cache_;
  std::map<uint8_t, int> report_strikes_;
  std::map<uint8_t, int> feature_strikes_;
  std::set<uint8_t> dead_reports_;        // GET_REPORT refuses/hangs on these
  std::set<uint8_t> dead_features_;
  std::set<uint8_t> interrupt_seen_;      // report IDs already announced
  bool interrupt_unsupported_{false};     // transport has no interrupt EP
  size_t map_lines_logged_{0};            // FIELD MAP stream cap accounting
};

}  // namespace ups_hid
}  // namespace esphome
