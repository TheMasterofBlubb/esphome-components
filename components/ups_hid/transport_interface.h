#pragma once

#include "esp_err.h"
#include <vector>
#include <cstdint>
#include <memory>

namespace esphome {
namespace ups_hid {

/**
 * Abstract USB Transport Interface
 * 
 * Provides a clean abstraction over USB communication, allowing for
 * different implementations (ESP32 hardware, simulation, etc.)
 * 
 * Design Pattern: Strategy Pattern for transport selection
 */
class IUsbTransport {
public:
    virtual ~IUsbTransport() = default;
    
    // Transport lifecycle
    virtual esp_err_t initialize() = 0;
    virtual esp_err_t deinitialize() = 0;
    
    // Device management
    virtual bool is_connected() const = 0;
    virtual uint16_t get_vendor_id() const = 0;
    virtual uint16_t get_product_id() const = 0;
    
    // HID communication
    virtual esp_err_t hid_get_report(uint8_t report_type, uint8_t report_id, 
                                   uint8_t* data, size_t* data_len, 
                                   uint32_t timeout_ms = 1000) = 0;
    
    virtual esp_err_t hid_set_report(uint8_t report_type, uint8_t report_id,
                                   const uint8_t* data, size_t data_len,
                                   uint32_t timeout_ms = 1000) = 0;
    
    // String descriptors
    virtual esp_err_t get_string_descriptor(uint8_t string_index, 
                                          std::string& result) = 0;
    
    // HID report descriptor (GET_DESCRIPTOR, bDescriptorType 0x22). Optional:
    // transports that cannot provide it return ESP_ERR_NOT_SUPPORTED so that
    // descriptor-driven protocols can fall back to the heuristic ones.
    virtual esp_err_t get_report_descriptor(std::vector<uint8_t>& descriptor,
                                            uint32_t timeout_ms = 1000) {
        (void) descriptor;
        (void) timeout_ms;
        return ESP_ERR_NOT_SUPPORTED;
    }

    // One-line USB topology summary (interfaces, HID class descriptor lengths,
    // which report-descriptor index was kept). Diagnostics only; empty when
    // the transport has none.
    virtual std::string get_usb_topology_summary() const { return std::string(); }

    // True while an asynchronous report-descriptor fetch is in flight
    // (get_report_descriptor returns ESP_ERR_INVALID_STATE meanwhile).
    virtual bool descriptor_fetch_pending() const { return false; }

    // Non-blocking-ish interrupt-IN poll. Transports without an interrupt
    // endpoint (or without support for this call) return NOT_SUPPORTED.
    virtual esp_err_t read_interrupt(uint8_t* data, size_t* len, uint32_t timeout_ms) {
        (void) data;
        (void) len;
        (void) timeout_ms;
        return ESP_ERR_NOT_SUPPORTED;
    }

    // Error information
    virtual std::string get_last_error() const = 0;
};

// Forward declaration - implementation in usb_transport_factory.h
class UsbTransportFactory;

} // namespace ups_hid
} // namespace esphome