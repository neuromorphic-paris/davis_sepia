#pragma once

#include "../third_party/sepia/source/sepia.hpp"
#include <array>
#include <libusb-1.0/libusb.h>

/// davis_sepia specialises sepia for the DAVIS 240c camera.
/// In order to use this header, an application must link to the dynamic library usb-1.0.
namespace davis_sepia {

    /// frame represents a timestamped frame.
    struct frame {
        /// t represents the event's timestamp.
        uint64_t t;

        /// exposure represents the exposure time for the frame.
        float exposure;

        /// pixels contains the frame's grey levels in row major order, starting from the bottom left.
        std::vector<uint16_t> pixels;
    };

    /// imu_event represents the parameters of an IMU measurement.
    SEPIA_PACK(struct imu_event {
        /// t represents the event's timestamp.
        uint64_t t;

        /// a_x is the acceleration's x component in m.s^-2.
        float a_x;

        /// a_y is the acceleration's y component in m.s^-2.
        float a_y;

        /// a_z is the acceleration's z component in m.s^-2.
        float a_z;

        /// temprature is the chip's temprature in °C.
        float temperature;

        /// w_x is the angular velocity's x component in rad.s^-1.
        float w_x;

        /// w_y is the angular velocity's y component in rad.s^-1.
        float w_y;

        /// w_z is the angular velocity's z component in rad.s^-1.
        float w_z;
    });

    /// input_type represents an event on an wire.
    enum class input_type {
        falling_edge,
        rising_edge,
        pulse,
    };

    /// external_input represents the parameters of an external input event.
    SEPIA_PACK(struct external_input {
        /// t represents the event's timestamp.
        uint64_t t;

        /// type is the detected external change type.
        input_type type;
    });

    /// camera represents a DAVIS camera.
    class camera {
        public:
        /// available_serials returns the connected DAVIS cameras' serials.
        static std::vector<std::string> available_serials() {
            std::vector<std::string> serials;
            libusb_context* context;
            check_usb_error(libusb_init(&context), "initializing the USB context");
            libusb_device** devices;
            const auto count = libusb_get_device_list(context, &devices);
            for (std::size_t index = 0; index < count; ++index) {
                libusb_device_descriptor descriptor;
                if (libusb_get_device_descriptor(devices[index], &descriptor) == 0) {
                    if (descriptor.idVendor == 5418 && descriptor.idProduct == 33819) {
                        libusb_device_handle* handle;
                        check_usb_error(libusb_open(devices[index], &handle), "opening the device");
                        if (libusb_claim_interface(handle, 0) == 0) {
                            std::array<uint8_t, 8> bytes;
                            auto bytes_read = libusb_get_string_descriptor_ascii(handle, 3, bytes.data(), bytes.size());
                            if (bytes_read <= 0) {
                                throw std::logic_error("retrieving the serial failed");
                            }
                            serials.push_back(std::string(bytes.data(), bytes.data() + bytes_read));
                        }
                        libusb_close(handle);
                    }
                }
            }
            libusb_free_device_list(devices, 1);
            libusb_exit(context);
            return serials;
        }

        /// default_parameter returns the default parameter used by the DAVIS.
        static std::unique_ptr<sepia::parameter> default_parameter() {
            return sepia::make_unique<sepia::object_parameter>(
                "mux",
                sepia::make_unique<sepia::object_parameter>(
                    "run",
                    sepia::make_unique<sepia::boolean_parameter>(true),
                    "timestamp_run",
                    sepia::make_unique<sepia::boolean_parameter>(true),
                    "timestamp_reset",
                    sepia::make_unique<sepia::boolean_parameter>(false),
                    "force_chip_bias_enable",
                    sepia::make_unique<sepia::boolean_parameter>(false),
                    "drop_dvs_on_transfer_stall",
                    sepia::make_unique<sepia::boolean_parameter>(true),
                    "drop_aps_on_transfer_stall",
                    sepia::make_unique<sepia::boolean_parameter>(false),
                    "drop_imu_on_transfer_stall",
                    sepia::make_unique<sepia::boolean_parameter>(false),
                    "drop_extinput_on_transfer_stall",
                    sepia::make_unique<sepia::boolean_parameter>(true)),
                "dvs",
                sepia::make_unique<sepia::object_parameter>(
                    "run",
                    sepia::make_unique<sepia::boolean_parameter>(true),
                    "ack_delay_row",
                    sepia::make_unique<sepia::number_parameter>(4, 0, 1ull << 32, true),
                    "ack_delay_column",
                    sepia::make_unique<sepia::number_parameter>(0, 0, 1ull << 32, true),
                    "ack_extension_row",
                    sepia::make_unique<sepia::number_parameter>(1, 0, 1ull << 32, true),
                    "ack_extension_column",
                    sepia::make_unique<sepia::number_parameter>(0, 0, 1ull << 32, true),
                    "wait_on_transfer_stall",
                    sepia::make_unique<sepia::boolean_parameter>(false),
                    "filter_row_only_events",
                    sepia::make_unique<sepia::boolean_parameter>(true),
                    "external_aer_control",
                    sepia::make_unique<sepia::boolean_parameter>(false)),
                "aps",
                sepia::make_unique<sepia::object_parameter>(
                    "run",
                    sepia::make_unique<sepia::boolean_parameter>(true),
                    "reset_read",
                    sepia::make_unique<sepia::boolean_parameter>(true),
                    "wait_on_transfer_stall",
                    sepia::make_unique<sepia::boolean_parameter>(true),
                    "global_shutter",
                    sepia::make_unique<sepia::boolean_parameter>(true),
                    "start_column_0",
                    sepia::make_unique<sepia::number_parameter>(0, 0, 240, true),
                    "start_row_0",
                    sepia::make_unique<sepia::number_parameter>(0, 0, 180, true),
                    "end_column_0",
                    sepia::make_unique<sepia::number_parameter>(239, 0, 240, true),
                    "end_row_0",
                    sepia::make_unique<sepia::number_parameter>(179, 0, 180, true),
                    "exposure",
                    sepia::make_unique<sepia::number_parameter>(20000 * 30, 0, 1 << 30, true),
                    "frame_delay",
                    sepia::make_unique<sepia::number_parameter>(1000 * 30, 0, 1 << 30, true),
                    "reset_settle",
                    sepia::make_unique<sepia::number_parameter>(30, 0, 1ull << 32, true),
                    "column_settle",
                    sepia::make_unique<sepia::number_parameter>(30, 0, 1ull << 32, true),
                    "row_settle",
                    sepia::make_unique<sepia::number_parameter>(10, 0, 1ull << 32, true),
                    "null_settle",
                    sepia::make_unique<sepia::number_parameter>(3, 0, 1ull << 32, true),
                    "roi0_enabled",
                    sepia::make_unique<sepia::boolean_parameter>(true)),
                "imu",
                sepia::make_unique<sepia::object_parameter>(
                    "run",
                    sepia::make_unique<sepia::boolean_parameter>(true),
                    "temp_standby",
                    sepia::make_unique<sepia::boolean_parameter>(false),
                    "accel_standby",
                    sepia::make_unique<sepia::boolean_parameter>(false),
                    "gyro_standby",
                    sepia::make_unique<sepia::boolean_parameter>(false),
                    "lp_cycle",
                    sepia::make_unique<sepia::boolean_parameter>(false),
                    "lp_wakeup",
                    sepia::make_unique<sepia::number_parameter>(1, 0, 4, true),
                    "sample_rate_divider",
                    sepia::make_unique<sepia::number_parameter>(0, 0, 1ull << 32, true),
                    "digital_low_pass_filter",
                    sepia::make_unique<sepia::number_parameter>(1, 0, 8, true),
                    "accel_full_scale",
                    sepia::make_unique<sepia::number_parameter>(1, 0, 4, true),
                    "gyro_full_scale",
                    sepia::make_unique<sepia::number_parameter>(1, 0, 4, true)),
                "extinput",
                sepia::make_unique<sepia::object_parameter>(
                    "run_detector",
                    sepia::make_unique<sepia::boolean_parameter>(true),
                    "detect_rising_edges",
                    sepia::make_unique<sepia::boolean_parameter>(false),
                    "detect_falling_edges",
                    sepia::make_unique<sepia::boolean_parameter>(false),
                    "detect_pulses",
                    sepia::make_unique<sepia::boolean_parameter>(true),
                    "detect_pulse_polarity",
                    sepia::make_unique<sepia::boolean_parameter>(true),
                    "detect_pulse_length",
                    sepia::make_unique<sepia::number_parameter>(90, 0, 1ull << 32, true)),
                "usb",
                sepia::make_unique<sepia::object_parameter>(
                    "run",
                    sepia::make_unique<sepia::boolean_parameter>(true),
                    "early_packet_delay",
                    sepia::make_unique<sepia::number_parameter>(8 * 125 * 30, 0, 1ull << 32, true)),
                "chip",
                sepia::make_unique<sepia::object_parameter>(
                    "digitalmux0",
                    sepia::make_unique<sepia::number_parameter>(0, 0, 1ull << 32, true),
                    "digitalmux1",
                    sepia::make_unique<sepia::number_parameter>(0, 0, 1ull << 32, true),
                    "digitalmux2",
                    sepia::make_unique<sepia::number_parameter>(0, 0, 1ull << 32, true),
                    "digitalmux3",
                    sepia::make_unique<sepia::number_parameter>(0, 0, 1ull << 32, true),
                    "analogmux0",
                    sepia::make_unique<sepia::number_parameter>(0, 0, 1ull << 32, true),
                    "analogmux1",
                    sepia::make_unique<sepia::number_parameter>(0, 0, 1ull << 32, true),
                    "analogmux2",
                    sepia::make_unique<sepia::number_parameter>(0, 0, 1ull << 32, true),
                    "biasmux0",
                    sepia::make_unique<sepia::number_parameter>(0, 0, 1ull << 32, true),
                    "resetcalibneuron",
                    sepia::make_unique<sepia::boolean_parameter>(true),
                    "typencalibneuron",
                    sepia::make_unique<sepia::boolean_parameter>(false),
                    "resettestpixel",
                    sepia::make_unique<sepia::boolean_parameter>(true),
                    "aernarow",
                    sepia::make_unique<sepia::boolean_parameter>(true),
                    "useaout",
                    sepia::make_unique<sepia::boolean_parameter>(true),
                    "global_shutter",
                    sepia::make_unique<sepia::boolean_parameter>(true)),
                "bias",
                sepia::make_unique<sepia::object_parameter>(
                    "diffbn",
                    sepia::make_unique<sepia::number_parameter>((1063 << 4) | 0b1111, 0, 1 << 16, true),
                    "onbn",
                    sepia::make_unique<sepia::number_parameter>((1535 << 4) | 0b1111, 0, 1 << 16, true),
                    "offbn",
                    sepia::make_unique<sepia::number_parameter>((1024 << 4) | 0b1111, 0, 1 << 16, true),
                    "apscasepc",
                    sepia::make_unique<sepia::number_parameter>((1465 << 4) | 0b1011, 0, 1 << 16, true),
                    "diffcasbnc",
                    sepia::make_unique<sepia::number_parameter>((1395 << 4) | 0b1011, 0, 1 << 16, true),
                    "apsrosfbn",
                    sepia::make_unique<sepia::number_parameter>((1755 << 4) | 0b1111, 0, 1 << 16, true),
                    "localbufbn",
                    sepia::make_unique<sepia::number_parameter>((1444 << 4) | 0b1111, 0, 1 << 16, true),
                    "pixinvbn",
                    sepia::make_unique<sepia::number_parameter>((1409 << 4) | 0b1111, 0, 1 << 16, true),
                    "prbp",
                    sepia::make_unique<sepia::number_parameter>((570 << 4) | 0b1101, 0, 1 << 16, true),
                    "prsfbp",
                    sepia::make_unique<sepia::number_parameter>((272 << 4) | 0b1101, 0, 1 << 16, true),
                    "refrbp",
                    sepia::make_unique<sepia::number_parameter>((1049 << 4) | 0b1101, 0, 1 << 16, true),
                    "aepdbn",
                    sepia::make_unique<sepia::number_parameter>((1627 << 4) | 0b1111, 0, 1 << 16, true),
                    "lcoltimeoutbn",
                    sepia::make_unique<sepia::number_parameter>((1329 << 4) | 0b1111, 0, 1 << 16, true),
                    "aepuxbp",
                    sepia::make_unique<sepia::number_parameter>((1104 << 4) | 0b1101, 0, 1 << 16, true),
                    "aepuybp",
                    sepia::make_unique<sepia::number_parameter>((1944 << 4) | 0b1101, 0, 1 << 16, true),
                    "ifthrbn",
                    sepia::make_unique<sepia::number_parameter>((1535 << 4) | 0b1111, 0, 1 << 16, true),
                    "ifrefrbn",
                    sepia::make_unique<sepia::number_parameter>((1535 << 4) | 0b1111, 0, 1 << 16, true),
                    "padfollbn",
                    sepia::make_unique<sepia::number_parameter>((2007 << 4) | 0b1111, 0, 1 << 16, true),
                    "apsoverflowlevelbn",
                    sepia::make_unique<sepia::number_parameter>((1789 << 4) | 0b1111, 0, 1 << 16, true),
                    "biasbuffer",
                    sepia::make_unique<sepia::number_parameter>((1534 << 4) | 0b1111, 0, 1 << 16, true),
                    "ssp",
                    sepia::make_unique<sepia::number_parameter>((33 << 10) | (1 << 4), 0, 1 << 16, true),
                    "ssn",
                    sepia::make_unique<sepia::number_parameter>((33 << 10) | (1 << 4), 0, 1 << 16, true)));
        }

        /// width returns the sensor width.
        static constexpr uint16_t width() {
            return 240;
        }

        /// height returns the sensor height.
        static constexpr uint16_t height() {
            return 180;
        }

        /// module_to_address associates parameters modules to their chip address.
        static std::unordered_map<std::string, uint16_t> module_to_address() {
            return {
                {"mux", 0},
                {"dvs", 1},
                {"aps", 2},
                {"imu", 3},
                {"extinput", 4},
                {"usb", 9},
                {"chip", 5},
                {"bias", 5},
            };
        }

        /// module_to_name_to_address associates parameters to their chip address.
        static std::unordered_map<std::string, std::unordered_map<std::string, uint16_t>> module_to_name_to_address() {
            return {
                {"mux",
                 {
                     {"timestamp_reset", 2},
                     {"force_chip_bias_enable", 3},
                     {"drop_dvs_on_transfer_stall", 4},
                     {"drop_aps_on_transfer_stall", 5},
                     {"drop_imu_on_transfer_stall", 6},
                     {"drop_extinput_on_transfer_stall", 7},
                 }},
                {"dvs",
                 {
                     {"ack_delay_row", 4},
                     {"ack_delay_column", 5},
                     {"ack_extension_row", 6},
                     {"ack_extension_column", 7},
                     {"wait_on_transfer_stall", 8},
                     {"filter_row_only_events", 9},
                     {"external_aer_control", 10},
                 }},
                {"aps",
                 {
                     {"reset_read", 5},
                     {"wait_on_transfer_stall", 6},
                     {"global_shutter", 8},
                     {"start_column_0", 9},
                     {"start_row_0", 10},
                     {"end_column_0", 11},
                     {"end_row_0", 12},
                     {"exposure", 13},
                     {"frame_delay", 14},
                     {"reset_settle", 15},
                     {"column_settle", 16},
                     {"row_settle", 17},
                     {"null_settle", 18},
                     {"roi0_enabled", 40},
                 }},
                {"imu",
                 {
                     {"temp_standby", 1},
                     {"accel_standby", 2},
                     {"gyro_standby", 3},
                     {"lp_cycle", 4},
                     {"lp_wakeup", 5},
                     {"sample_rate_divider", 6},
                     {"digital_low_pass_filter", 7},
                     {"accel_full_scale", 8},
                     {"gyro_full_scale", 9},
                 }},
                {"extinput",
                 {
                     {"detect_rising_edges", 1},
                     {"detect_falling_edges", 2},
                     {"detect_pulses", 3},
                     {"detect_pulse_polarity", 4},
                     {"detect_pulse_length", 5},
                 }},
                {"usb",
                 {
                     {"early_packet_delay", 1},
                 }},
                {"chip",
                 {
                     {"digitalmux0", 128},
                     {"digitalmux1", 129},
                     {"digitalmux2", 130},
                     {"digitalmux3", 131},
                     {"analogmux0", 132},
                     {"analogmux1", 133},
                     {"analogmux2", 134},
                     {"biasmux0", 135},
                     {"resetcalibneuron", 136},
                     {"typencalibneuron", 137},
                     {"resettestpixel", 138},
                     {"aernarow", 140},
                     {"useaout", 141},
                     {"global_shutter", 142},
                 }},
                {"bias",
                 {
                     {"diffbn", 0},
                     {"onbn", 1},
                     {"offbn", 2},
                     {"apscasepc", 3},
                     {"diffcasbnc", 4},
                     {"apsrosfbn", 5},
                     {"localbufbn", 6},
                     {"pixinvbn", 7},
                     {"prbp", 8},
                     {"prsfbp", 9},
                     {"refrbp", 10},
                     {"aepdbn", 11},
                     {"lcoltimeoutbn", 12},
                     {"aepuxbp", 13},
                     {"aepuybp", 14},
                     {"ifthrbn", 15},
                     {"ifrefrbn", 16},
                     {"padfollbn", 17},
                     {"apsoverflowlevelbn", 18},
                     {"biasbuffer", 19},
                     {"ssp", 20},
                     {"ssn", 21},
                 }},
            };
        }

        /// run_module_to_name_to_address associates parameters that start chip systems to their chip address.
        static std::unordered_map<std::string, std::unordered_map<std::string, uint16_t>>
        run_module_to_name_to_address() {
            return {
                {"dvs",
                 {
                     {"run", 3},
                 }},
                {"aps",
                 {
                     {"run", 4},
                 }},
                {"imu",
                 {
                     {"run", 0},
                 }},
                {"extinput",
                 {
                     {"run_detector", 0},
                 }},
            };
        }

        /// transfer_module_to_name_to_address associates parameters that start data transfer to their chip address.
        static std::unordered_map<std::string, std::unordered_map<std::string, uint16_t>>
        transfer_module_to_name_to_address() {
            return {
                {"usb",
                 {
                     {"run", 0},
                 }},
                {"mux",
                 {
                     {"run", 0},
                     {"timestamp_run", 1},
                 }},
            };
        }

        camera() = default;
        camera(const camera&) = delete;
        camera(camera&&) = default;
        camera& operator=(const camera&) = delete;
        camera& operator=(camera&&) = default;
        virtual ~camera() {}

        protected:
        /// check_usb_error throws if the given value is not zero.
        static void check_usb_error(int error, const std::string& message) {
            if (error < 0) {
                throw std::logic_error(message + " failed: " + libusb_strerror(static_cast<libusb_error>(error)));
            }
        }
    };

    /// specialized_camera represents a template-specialized DVS128.
    template <
        typename HandleEvent,
        typename HandleFrame,
        typename HandleImuEvent,
        typename HandleExternalInput,
        typename HandleException>
    class specialized_camera : public camera {
        public:
        specialized_camera(
            HandleEvent handle_event,
            HandleFrame handle_frame,
            HandleImuEvent handle_imu_event,
            HandleExternalInput handle_external_input,
            HandleException handle_exception,
            std::unique_ptr<sepia::unvalidated_parameter> unvalidated_parameter,
            std::size_t events_fifo_size,
            std::size_t frames_fifo_size,
            std::size_t imu_events_fifo_size,
            const std::string& serial,
            std::chrono::milliseconds sleep_duration) :
            _handle_event(std::forward<HandleEvent>(handle_event)),
            _handle_frame(std::forward<HandleFrame>(handle_frame)),
            _handle_imu_event(std::forward<HandleImuEvent>(handle_imu_event)),
            _handle_external_input(std::forward<HandleExternalInput>(handle_external_input)),
            _handle_exception(std::forward<HandleException>(handle_exception)),
            _buffer_running(true),
            _sleep_duration(sleep_duration),
            _events_head(0),
            _events_tail(0),
            _events(events_fifo_size),
            _frames_head(0),
            _frames_tail(0),
            _frames(
                frames_fifo_size,
                {0, 0, std::vector<uint16_t>(static_cast<std::size_t>(camera::width()) * camera::height())}),
            _imu_events_head(0),
            _imu_events_tail(0),
            _imu_events(imu_events_fifo_size),
            _parameter(default_parameter()),
            _acquisition_running(true) {
            _parameter->parse_or_load(std::move(unvalidated_parameter));

            // initialize the context
            check_usb_error(libusb_init(&_context), "initializing the USB context");

            // find requested / available devices
            {
                auto device_found = false;
                libusb_device** devices;
                const auto count = libusb_get_device_list(_context, &devices);
                for (std::size_t index = 0; index < count; ++index) {
                    libusb_device_descriptor descriptor;
                    if (libusb_get_device_descriptor(devices[index], &descriptor) == 0) {
                        if (descriptor.idVendor == 5418 && descriptor.idProduct == 33819) {
                            check_usb_error(libusb_open(devices[index], &_handle), "opening the device");
                            if (libusb_claim_interface(_handle, 0) == 0) {
                                if (serial.empty()) {
                                    device_found = true;
                                    break;
                                } else {
                                    std::array<uint8_t, 8> bytes;
                                    auto bytes_read =
                                        libusb_get_string_descriptor_ascii(_handle, 3, bytes.data(), bytes.size());
                                    if (bytes_read <= 0) {
                                        throw std::logic_error("retrieving the serial failed");
                                    }
                                    if (std::string(bytes.data(), bytes.data() + bytes_read) == serial) {
                                        device_found = true;
                                        break;
                                    }
                                }
                                libusb_release_interface(_handle, 0);
                            }
                            libusb_close(_handle);
                        }
                    }
                }
                libusb_free_device_list(devices, 1);
                if (!device_found) {
                    libusb_exit(_context);
                    throw sepia::no_device_connected("DAVIS");
                }
            }

            // allocate a transfer
            _transfer = libusb_alloc_transfer(0);

            // send setup commands to the camera
            check_usb_error(libusb_reset_device(_handle), "resetting the device");
            auto send = [this](
                            const std::string& module_name,
                            uint16_t module,
                            const std::string& address_name,
                            uint16_t address,
                            uint32_t value) {
                std::array<uint8_t, 4> bytes{static_cast<uint8_t>(value >> 24),
                                             static_cast<uint8_t>(value >> 16),
                                             static_cast<uint8_t>(value >> 8),
                                             static_cast<uint8_t>(value & 0xff)};
                check_usb_error(
                    libusb_control_transfer(_handle, 64, 191, module, address, bytes.data(), bytes.size(), 0),
                    std::string("sending the control packet to ") + module_name + "::" + address_name);
            };
            for (const std::string& module :
                 std::vector<std::string>{"bias", "chip", "mux", "dvs", "aps", "imu", "extinput", "usb"}) {
                const auto name_to_address = module_to_name_to_address().at(module);
                const auto module_address = module_to_address().at(module);
                for (const auto& name_and_address : name_to_address) {
                    uint32_t value;
                    try {
                        value = static_cast<uint32_t>(_parameter->get_number({module, name_and_address.first}));
                    } catch (const sepia::parameter_error&) {
                        value = _parameter->get_boolean({module, name_and_address.first}) ? 1 : 0;
                    }
                    send(module, module_address, name_and_address.first, name_and_address.second, value);
                }
            }
            for (const std::string& module : std::vector<std::string>{"dvs", "aps", "imu", "extinput"}) {
                const auto name_to_address = run_module_to_name_to_address().at(module);
                const auto module_address = module_to_address().at(module);
                for (const auto& name_and_address : name_to_address) {
                    send(
                        module,
                        module_address,
                        name_and_address.first,
                        name_and_address.second,
                        _parameter->get_boolean({module, name_and_address.first}) ? 1 : 0);
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            for (const std::string& module : std::vector<std::string>{"usb", "mux"}) {
                const auto name_to_address = transfer_module_to_name_to_address().at(module);
                const auto module_address = module_to_address().at(module);
                for (const auto& name_and_address : name_to_address) {
                    send(
                        module,
                        module_address,
                        name_and_address.first,
                        name_and_address.second,
                        _parameter->get_boolean({module, name_and_address.first}) ? 1 : 0);
                }
            }

            // start the event handling loop
            _buffer_loop = std::thread([this]() -> void {
                try {
                    sepia::dvs_event event = {};
                    frame buffered_frame = {
                        0, 0, std::vector<uint16_t>(static_cast<std::size_t>(camera::width()) * camera::height())};
                    imu_event buffered_imu_event = {};
                    while (_buffer_running.load(std::memory_order_relaxed)) {
                        const auto current_head = _events_head.load(std::memory_order_relaxed);
                        if (current_head == _events_tail.load(std::memory_order_acquire)) {
                            std::this_thread::sleep_for(_sleep_duration);
                        } else {
                            event = _events[current_head];
                            _events_head.store((current_head + 1) % _events.size(), std::memory_order_release);
                            switch (event.x) {
                                case 240:
                                    while (_buffer_running.load(std::memory_order_relaxed)) {
                                        const auto current_frames_head = _frames_head.load(std::memory_order_relaxed);
                                        if (current_frames_head == _frames_tail.load(std::memory_order_acquire)) {
                                            std::this_thread::sleep_for(_sleep_duration);
                                        } else {
                                            buffered_frame.t = _frames[current_frames_head].t;
                                            buffered_frame.exposure = _frames[current_frames_head].exposure;
                                            buffered_frame.pixels.swap(_frames[current_frames_head].pixels);
                                            _frames_head.store(
                                                (current_frames_head + 1) % _frames.size(), std::memory_order_release);
                                            this->_handle_frame(buffered_frame);
                                            break;
                                        }
                                    }
                                    break;
                                case 241:
                                    while (_buffer_running.load(std::memory_order_relaxed)) {
                                        const auto current_imu_events_head =
                                            _imu_events_head.load(std::memory_order_relaxed);
                                        if (current_imu_events_head
                                            == _imu_events_tail.load(std::memory_order_acquire)) {
                                            std::this_thread::sleep_for(_sleep_duration);
                                        } else {
                                            buffered_imu_event = _imu_events[current_imu_events_head];
                                            _imu_events_head.store(
                                                (current_imu_events_head + 1) % _imu_events.size(),
                                                std::memory_order_release);
                                            this->_handle_imu_event(buffered_imu_event);
                                            break;
                                        }
                                    }
                                    break;
                                case 242:
                                    this->_handle_external_input(external_input{event.t, input_type::falling_edge});
                                    break;
                                case 243:
                                    this->_handle_external_input(external_input{event.t, input_type::rising_edge});
                                    break;
                                case 244:
                                    this->_handle_external_input(external_input{event.t, input_type::pulse});
                                    break;
                                default:
                                    this->_handle_event(event);
                            }
                        }
                    }
                } catch (...) {
                    this->_handle_exception(std::current_exception());
                }
            });

            // start the reading loop
            _acquisition_loop = std::thread([this, serial]() -> void {
                try {
                    std::vector<uint8_t> bytes(8192);
                    uint64_t t_offset = 0;
                    sepia::dvs_event event = {};
                    frame buffered_frame = {
                        0, 0, std::vector<uint16_t>(static_cast<std::size_t>(camera::width()) * camera::height())};
                    auto frame_started = false;
                    uint8_t frame_region_of_interest_first_byte = 0;
                    uint8_t frame_region_of_interest_index = 255;
                    std::array<uint16_t, 4> frame_region_of_interest = {0, 0, width(), height()};
                    std::array<uint16_t, 4> new_frame_region_of_interest;
                    uint8_t frame_x = 0;
                    uint8_t frame_y = 0;
                    auto is_frame_reset = false;
                    uint32_t frame_exposure = 0;
                    uint8_t frame_exposure_count = 255;
                    imu_event buffered_imu_event = {};
                    auto imu_started = false;
                    auto imu_a_scale = 9.81f * 2 / 32768;
                    auto imu_w_scale = static_cast<float>(M_PI) / 180 * 250 / 32768;
                    uint8_t imu_first_byte = 0;
                    uint8_t imu_byte_index = 255;
                    auto imu_bytes_to_float = [](uint8_t first_byte, uint8_t second_byte) -> float {
                        int16_t value = second_byte;
                        *(reinterpret_cast<uint8_t*>(&value) + 1) = first_byte;
                        return value;
                    };
                    while (_acquisition_running.load(std::memory_order_relaxed)) {
                        int32_t transferred = 0;
                        const auto error = libusb_bulk_transfer(
                            _handle, 130, bytes.data(), static_cast<uint32_t>(bytes.size()), &transferred, 0);
                        if (error == 0 || error == LIBUSB_ERROR_TIMEOUT) {
                            if (transferred % 2 != 0) {
                                --transferred;
                            }
                            for (auto byte_iterator = bytes.begin();
                                 byte_iterator != std::next(bytes.begin(), transferred);
                                 std::advance(byte_iterator, 2)) {
                                const uint8_t type = (*std::next(byte_iterator) >> 4);
                                const uint16_t data =
                                    ((static_cast<uint16_t>(*std::next(byte_iterator) & 0xf) << 8) | *byte_iterator);
                                if ((*std::next(byte_iterator) >> 7) == 1) {
                                    const uint64_t new_t = t_offset + data;
                                    if (new_t > event.t) {
                                        event.t = new_t;
                                    }
                                } else {
                                    switch (type) {
                                        case 0:
                                            switch (data) {
                                                case 1:
                                                    event.t = 0;
                                                    t_offset = 0;
                                                    break;
                                                case 2:
                                                case 3:
                                                case 4: {
                                                    event.x = 240 + (*std::next(byte_iterator) >> 4);
                                                    const auto current_tail =
                                                        _events_tail.load(std::memory_order_relaxed);
                                                    const auto next_tail = (current_tail + 1) % _events.size();
                                                    if (next_tail != _events_head.load(std::memory_order_acquire)) {
                                                        _events[current_tail] = event;
                                                        _events_tail.store(next_tail, std::memory_order_release);
                                                    } else {
                                                        throw std::runtime_error("computer's event FIFO overflow");
                                                    }
                                                    break;
                                                }
                                                case 5:
                                                    buffered_imu_event = {0, 0, 0, 0, 0, 0, 0, 0};
                                                    imu_first_byte = 0;
                                                    imu_byte_index = 0;
                                                    imu_started = true;
                                                    break;
                                                case 7:
                                                    if (imu_started) {
                                                        buffered_imu_event.t = event.t;
                                                        const auto current_imu_events_tail =
                                                            _imu_events_tail.load(std::memory_order_relaxed);
                                                        const auto next_imu_events_tail =
                                                            (current_imu_events_tail + 1) % _imu_events.size();
                                                        if (next_imu_events_tail
                                                            != _imu_events_head.load(std::memory_order_acquire)) {
                                                            _imu_events[current_imu_events_tail] = buffered_imu_event;
                                                            _imu_events_tail.store(
                                                                next_imu_events_tail, std::memory_order_release);
                                                        } else {
                                                            throw std::runtime_error(
                                                                "computer's IMU event FIFO overflow");
                                                        }
                                                        event.x = 241;
                                                        const auto current_tail =
                                                            _events_tail.load(std::memory_order_relaxed);
                                                        const auto next_tail = (current_tail + 1) % _events.size();
                                                        if (next_tail != _events_head.load(std::memory_order_acquire)) {
                                                            _events[current_tail] = event;
                                                            _events_tail.store(next_tail, std::memory_order_release);
                                                        } else {
                                                            throw std::runtime_error("computer's event FIFO overflow");
                                                        }
                                                        imu_started = false;
                                                    }
                                                    break;
                                                case 8:
                                                case 9:
                                                case 14:
                                                case 15:
                                                    std::fill(
                                                        buffered_frame.pixels.begin(), buffered_frame.pixels.end(), 0);
                                                    if (frame_region_of_interest[0] <= frame_region_of_interest[2]) {
                                                        for (uint16_t y = frame_region_of_interest[1];
                                                             y <= frame_region_of_interest[3];
                                                             ++y) {
                                                            std::fill(
                                                                std::next(
                                                                    buffered_frame.pixels.begin(),
                                                                    frame_region_of_interest[0]
                                                                        + static_cast<std::size_t>(height() - 1 - y)
                                                                              * width()),
                                                                std::next(
                                                                    buffered_frame.pixels.begin(),
                                                                    frame_region_of_interest[2]
                                                                        + static_cast<std::size_t>(height() - 1 - y)
                                                                              * width()),
                                                                ((1 << 10) - 1) << 6);
                                                        }
                                                    }
                                                    frame_started = true;
                                                    frame_x = frame_region_of_interest[0];
                                                    break;
                                                case 10:
                                                    if (frame_started) {
                                                        buffered_frame.t = event.t;
                                                        buffered_frame.exposure =
                                                            static_cast<float>(frame_exposure) / 30;
                                                        const auto current_frames_tail =
                                                            _frames_tail.load(std::memory_order_relaxed);
                                                        const auto next_frames_tail =
                                                            (current_frames_tail + 1) % _frames.size();
                                                        if (next_frames_tail
                                                            != _frames_head.load(std::memory_order_acquire)) {
                                                            _frames[current_frames_tail].t = buffered_frame.t;
                                                            _frames[current_frames_tail].exposure =
                                                                buffered_frame.exposure;
                                                            _frames[current_frames_tail].pixels.swap(
                                                                buffered_frame.pixels);
                                                            _frames_tail.store(
                                                                next_frames_tail, std::memory_order_release);
                                                        } else {
                                                            throw std::runtime_error("computer's frame FIFO overflow");
                                                        }
                                                        event.x = 240;
                                                        const auto current_tail =
                                                            _events_tail.load(std::memory_order_relaxed);
                                                        const auto next_tail = (current_tail + 1) % _events.size();
                                                        if (next_tail != _events_head.load(std::memory_order_acquire)) {
                                                            _events[current_tail] = event;
                                                            _events_tail.store(next_tail, std::memory_order_release);
                                                        } else {
                                                            throw std::runtime_error("computer's event FIFO overflow");
                                                        }
                                                        frame_started = false;
                                                    }
                                                    break;
                                                case 11:
                                                    is_frame_reset = true;
                                                    break;
                                                case 12:
                                                    if (is_frame_reset) {
                                                        frame_x = frame_region_of_interest[0];
                                                    }
                                                    is_frame_reset = false;
                                                    break;
                                                case 13:
                                                    ++frame_x;
                                                    frame_y = frame_region_of_interest[1];
                                                    break;
                                                case 16:
                                                case 17:
                                                case 18:
                                                case 19:
                                                case 20:
                                                case 21:
                                                case 22:
                                                case 23:
                                                case 24:
                                                case 25:
                                                case 26:
                                                case 27:
                                                case 28:
                                                case 29:
                                                case 30:
                                                case 31:
                                                    if (imu_started) {
                                                        imu_a_scale =
                                                            9.81f * 2 * (1 << ((*byte_iterator >> 2) & 0b11)) / 32768;
                                                        imu_w_scale = static_cast<float>(M_PI) / 180 * 250
                                                                      * (1 << (*byte_iterator & 0b11)) / 32768;
                                                    }
                                                    break;
                                                case 32:
                                                    frame_region_of_interest_first_byte = 0;
                                                    frame_region_of_interest_index = 0;
                                                    break;
                                                case 48:
                                                    frame_exposure = 0;
                                                    frame_exposure_count = 0;
                                                    break;
                                                default:
                                                    break;
                                            }
                                            break;
                                        case 1:
                                            if (data < height()) {
                                                event.y = height() - 1 - data;
                                            }
                                            break;
                                        case 2:
                                        case 3:
                                            if (data < width()) {
                                                event.x = data;
                                                event.is_increase = (type == 3);
                                                const auto current_tail = _events_tail.load(std::memory_order_relaxed);
                                                const auto next_tail = (current_tail + 1) % _events.size();
                                                if (next_tail != _events_head.load(std::memory_order_acquire)) {
                                                    _events[current_tail] = event;
                                                    _events_tail.store(next_tail, std::memory_order_release);
                                                } else {
                                                    throw std::runtime_error("computer's event FIFO overflow");
                                                }
                                            }
                                            break;
                                        case 4:
                                            if (frame_started && frame_x <= frame_region_of_interest[2]
                                                && frame_y <= frame_region_of_interest[3]) {
                                                const auto index =
                                                    frame_x
                                                    + static_cast<std::size_t>(height() - 1 - frame_y) * width();
                                                if (is_frame_reset) {
                                                    buffered_frame.pixels[index] = (data << 6);
                                                } else {
                                                    if ((data << 6) < buffered_frame.pixels[index]) {
                                                        buffered_frame.pixels[index] -= (data << 6);
                                                    } else {
                                                        buffered_frame.pixels[index] = 0;
                                                    }
                                                }
                                                ++frame_y;
                                            }
                                            break;
                                        case 5: {
                                            switch (*std::next(byte_iterator) & 0xf) {
                                                case 0:
                                                    if (imu_byte_index < 14) {
                                                        switch (imu_byte_index) {
                                                            case 1:
                                                                buffered_imu_event.a_x =
                                                                    imu_bytes_to_float(imu_first_byte, *byte_iterator)
                                                                    * imu_a_scale;
                                                                break;
                                                            case 3:
                                                                buffered_imu_event.a_y =
                                                                    imu_bytes_to_float(imu_first_byte, *byte_iterator)
                                                                    * imu_a_scale;
                                                                break;
                                                            case 5:
                                                                buffered_imu_event.a_z =
                                                                    imu_bytes_to_float(imu_first_byte, *byte_iterator)
                                                                    * imu_a_scale;
                                                                break;
                                                            case 7:
                                                                buffered_imu_event.temperature =
                                                                    imu_bytes_to_float(imu_first_byte, *byte_iterator)
                                                                        / 340
                                                                    + 36.53;
                                                                break;
                                                            case 9:
                                                                buffered_imu_event.w_x =
                                                                    imu_bytes_to_float(imu_first_byte, *byte_iterator)
                                                                    * imu_w_scale;
                                                                break;
                                                            case 11:
                                                                buffered_imu_event.w_y =
                                                                    imu_bytes_to_float(imu_first_byte, *byte_iterator)
                                                                    * imu_w_scale;
                                                                break;
                                                            case 13:
                                                                buffered_imu_event.w_z =
                                                                    imu_bytes_to_float(imu_first_byte, *byte_iterator)
                                                                    * imu_w_scale;
                                                                break;
                                                            default:
                                                                imu_first_byte = *byte_iterator;
                                                        }
                                                        ++imu_byte_index;
                                                    }
                                                    break;
                                                case 1:
                                                    if (frame_region_of_interest_index < 4) {
                                                        frame_region_of_interest_first_byte = *byte_iterator;
                                                    }
                                                    break;
                                                case 2:
                                                    if (frame_region_of_interest_index < 4) {
                                                        const uint16_t value =
                                                            (static_cast<uint16_t>(frame_region_of_interest_first_byte)
                                                             << 8)
                                                            | *byte_iterator;
                                                        if (frame_region_of_interest_index % 2 == 0) {
                                                            if (value < width()) {
                                                                new_frame_region_of_interest
                                                                    [frame_region_of_interest_index] = value;
                                                            }
                                                        } else {
                                                            if (value < height()) {
                                                                new_frame_region_of_interest
                                                                    [frame_region_of_interest_index] = value;
                                                            }
                                                        }
                                                        if (frame_region_of_interest_index == 3) {
                                                            frame_region_of_interest = new_frame_region_of_interest;
                                                        }
                                                        ++frame_region_of_interest_index;
                                                    }
                                                    break;
                                                default:
                                                    break;
                                            }
                                            break;
                                        }
                                        case 6:
                                            if ((*std::next(byte_iterator) & 0x0b1100) == 0
                                                && frame_exposure_count < 2) {
                                                frame_exposure |= ((data & 0x03ff) << (10 * frame_exposure_count));
                                                ++frame_exposure_count;
                                            }
                                            break;
                                        case 7:
                                            t_offset += 0x8000;
                                            break;
                                        default:
                                            break;
                                    }
                                }
                            }
                        } else {
                            throw sepia::device_disconnected("DAVIS");
                        }
                    }
                } catch (...) {
                    this->_handle_exception(std::current_exception());
                }
            });
        }
        specialized_camera(const specialized_camera&) = delete;
        specialized_camera(specialized_camera&&) = default;
        specialized_camera& operator=(const specialized_camera&) = delete;
        specialized_camera& operator=(specialized_camera&&) = default;
        virtual ~specialized_camera() {
            _acquisition_running.store(false, std::memory_order_relaxed);
            if (_acquisition_loop.joinable()) {
                _acquisition_loop.join();
            }
            libusb_release_interface(_handle, 0);
            libusb_free_transfer(_transfer);
            libusb_close(_handle);
            libusb_exit(_context);
            _buffer_running.store(false, std::memory_order_relaxed);
            _buffer_loop.join();
        }

        protected:
        HandleEvent _handle_event;
        HandleFrame _handle_frame;
        HandleImuEvent _handle_imu_event;
        HandleExternalInput _handle_external_input;
        HandleException _handle_exception;
        std::thread _buffer_loop;
        std::atomic_bool _buffer_running;
        const std::chrono::milliseconds _sleep_duration;
        std::atomic<std::size_t> _events_head;
        std::atomic<std::size_t> _events_tail;
        std::vector<sepia::dvs_event> _events;
        std::atomic<std::size_t> _frames_head;
        std::atomic<std::size_t> _frames_tail;
        std::vector<frame> _frames;
        std::atomic<std::size_t> _imu_events_head;
        std::atomic<std::size_t> _imu_events_tail;
        std::vector<imu_event> _imu_events;
        std::unique_ptr<sepia::parameter> _parameter;
        std::atomic_bool _acquisition_running;
        libusb_context* _context;
        libusb_device_handle* _handle;
        libusb_transfer* _transfer;
        std::thread _acquisition_loop;
    };

    /// make_camera creates a camera from functors.
    template <
        typename HandleEvent,
        typename HandleFrame,
        typename HandleImuEvent,
        typename HandleExternalInput,
        typename HandleException>
    std::unique_ptr<specialized_camera<HandleEvent, HandleFrame, HandleImuEvent, HandleExternalInput, HandleException>>
    make_camera(
        HandleEvent handle_event,
        HandleFrame handle_frame,
        HandleImuEvent handle_imu_event,
        HandleExternalInput handle_external_input,
        HandleException handle_exception,
        std::unique_ptr<sepia::unvalidated_parameter> unvalidated_parameter =
            std::unique_ptr<sepia::unvalidated_parameter>(),
        std::size_t events_fifo_size = 1 << 24,
        std::size_t frames_fifo_size = 1 << 10,
        std::size_t imu_events_fifo_size = 1 << 16,
        const std::string& serial = std::string(),
        std::chrono::milliseconds sleep_duration = std::chrono::milliseconds(10)) {
        return sepia::make_unique<
            specialized_camera<HandleEvent, HandleFrame, HandleImuEvent, HandleExternalInput, HandleException>>(
            std::forward<HandleEvent>(handle_event),
            std::forward<HandleFrame>(handle_frame),
            std::forward<HandleImuEvent>(handle_imu_event),
            std::forward<HandleExternalInput>(handle_external_input),
            std::forward<HandleException>(handle_exception),
            std::move(unvalidated_parameter),
            events_fifo_size,
            frames_fifo_size,
            imu_events_fifo_size,
            serial,
            sleep_duration);
    }
}
