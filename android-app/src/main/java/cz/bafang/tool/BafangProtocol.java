package cz.bafang.tool;

import com.hoho.android.usbserial.driver.UsbSerialPort;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.util.Arrays;
import java.util.Locale;

final class BafangProtocol {
    static final int BAUD_RATE = 1200;
    private final UsbSerialPort port;
    private DeviceInfo deviceInfo;

    BafangProtocol(UsbSerialPort port) {
        this.port = port;
    }

    DeviceInfo getDeviceInfo() {
        return deviceInfo;
    }

    boolean connect() throws IOException, InterruptedException {
        port.setParameters(BAUD_RATE, 8, UsbSerialPort.STOPBITS_1, UsbSerialPort.PARITY_NONE);
        try {
            port.setDTR(true);
            port.setRTS(true);
        } catch (UnsupportedOperationException ignored) {
        }
        Thread.sleep(300);
        byte[] response = send(new byte[] {0x11, 0x51, 0x04, (byte) 0xB0, 0x05}, 300);
        if (response.length < 19 || u(response[0]) != 0x51) {
            return false;
        }
        deviceInfo = DeviceInfo.fromResponse(response);
        return true;
    }

    SectionResult readBasic() throws IOException, InterruptedException {
        return readSection("Basic", 0x52, 150);
    }

    SectionResult readPedal() throws IOException, InterruptedException {
        return readSection("Pedal", 0x53, 150);
    }

    SectionResult readThrottle() throws IOException, InterruptedException {
        return readSection("Throttle", 0x54, 150);
    }

    SectionResult readLiveData() throws IOException, InterruptedException {
        return readSection("Live data", 0x19, 150);
    }

    SectionResult readErrors() throws IOException, InterruptedException {
        return readSection("Errors", 0x1A, 150);
    }

    private SectionResult readSection(String name, int code, int waitMs) throws IOException, InterruptedException {
        byte[] cmd = new byte[] {(byte) code, 0x00};
        byte[] response = send(cmd, waitMs);
        return SectionResult.fromResponse(name, code, response);
    }

    private byte[] send(byte[] command, int waitMs) throws IOException, InterruptedException {
        port.write(command, 2000);
        Thread.sleep(waitMs);
        byte[] buffer = new byte[2048];
        int len = port.read(buffer, 2000);
        if (len <= 0) {
            return new byte[0];
        }
        return Arrays.copyOf(buffer, len);
    }

    static int u(byte value) {
        return value & 0xFF;
    }

    static String hex(byte[] bytes) {
        StringBuilder out = new StringBuilder(bytes.length * 3);
        for (byte b : bytes) {
            if (out.length() > 0) {
                out.append(' ');
            }
            out.append(String.format(Locale.US, "%02X", u(b)));
        }
        return out.toString();
    }

    static final class DeviceInfo {
        final String manufacturer;
        final String model;
        final String hardwareVersion;
        final String voltage;
        final int maxCurrent;
        final String raw;

        private DeviceInfo(String manufacturer, String model, String hardwareVersion, String voltage, int maxCurrent, String raw) {
            this.manufacturer = manufacturer;
            this.model = model;
            this.hardwareVersion = hardwareVersion;
            this.voltage = voltage;
            this.maxCurrent = maxCurrent;
            this.raw = raw;
        }

        static DeviceInfo fromResponse(byte[] response) {
            byte[] payload = payload(response);
            String manufacturer = ascii(payload, 0, 4);
            String model = ascii(payload, 4, 8);
            String hw = payload.length > 9 ? ((char) u(payload[8])) + "." + ((char) u(payload[9])) : "";
            int voltageCode = payload.length > 14 ? u(payload[14]) : -1;
            int current = payload.length > 15 ? u(payload[15]) : 0;
            String voltage;
            switch (voltageCode) {
                case 0: voltage = "24V"; break;
                case 1: voltage = "36V"; break;
                case 2: voltage = "48V"; break;
                case 3: voltage = "43V"; break;
                case 4: voltage = "24V-48V"; break;
                default: voltage = "Unknown"; break;
            }
            return new DeviceInfo(manufacturer, model, hw, voltage, current, hex(response));
        }

        String format() {
            return "Výrobce: " + manufacturer + "\n"
                    + "Model: " + model + "\n"
                    + "HW: " + hardwareVersion + "\n"
                    + "Napětí: " + voltage + "\n"
                    + "Max proud: " + maxCurrent + " A\n"
                    + "Raw connect: " + raw;
        }
    }

    static final class SectionResult {
        final String name;
        final boolean ok;
        final String summary;
        final String raw;

        private SectionResult(String name, boolean ok, String summary, String raw) {
            this.name = name;
            this.ok = ok;
            this.summary = summary;
            this.raw = raw;
        }

        static SectionResult fromResponse(String name, int expectedCode, byte[] response) {
            if (response.length == 0) {
                return new SectionResult(name, false, "Bez odpovědi", "");
            }
            if (u(response[0]) != expectedCode) {
                return new SectionResult(name, false, "Neočekávaná odpověď 0x" + String.format(Locale.US, "%02X", u(response[0])), hex(response));
            }
            byte[] payload = payload(response);
            return new SectionResult(name, true, summarize(name, payload), hex(response));
        }

        String format() {
            return name + ": " + (ok ? "OK" : "CHYBA") + "\n" + summary + "\nRaw: " + raw;
        }
    }

    private static byte[] payload(byte[] response) {
        if (response.length <= 2) {
            return new byte[0];
        }
        int declared = u(response[1]);
        int available = response.length - 2;
        int len = Math.min(declared > 0 ? declared : available, available);
        return Arrays.copyOfRange(response, 2, 2 + len);
    }

    private static String summarize(String name, byte[] payload) {
        if (payload.length == 0) {
            return "Prázdný payload";
        }
        if ("Basic".equals(name)) {
            return "Délka: " + payload.length + " B\n"
                    + valueLine("Low battery", payload, 0, " V")
                    + valueLine("Max current", payload, 1, " A")
                    + valueLine("Speed limit", payload, 2, " km/h");
        }
        if ("Pedal".equals(name)) {
            return "Délka: " + payload.length + " B\n"
                    + valueLine("Designated assist", payload, 1, "")
                    + valueLine("Speed limit", payload, 2, "")
                    + valueLine("Start current", payload, 9, " %");
        }
        if ("Throttle".equals(name)) {
            return "Délka: " + payload.length + " B\n"
                    + valueLine("Mode", payload, 0, "")
                    + valueLine("Assist level", payload, 1, "")
                    + valueLine("Speed limit", payload, 2, "");
        }
        if ("Live data".equals(name)) {
            return "Délka: " + payload.length + " B\n" + hex(payload);
        }
        if ("Errors".equals(name)) {
            return "Délka: " + payload.length + " B\n" + hex(payload);
        }
        return "Délka: " + payload.length + " B\n" + hex(payload);
    }

    private static String valueLine(String label, byte[] payload, int index, String suffix) {
        if (payload.length <= index) {
            return label + ": n/a\n";
        }
        return label + ": " + u(payload[index]) + suffix + "\n";
    }

    private static String ascii(byte[] payload, int start, int end) {
        if (payload.length <= start) {
            return "";
        }
        int safeEnd = Math.min(end, payload.length);
        return new String(payload, start, safeEnd - start, StandardCharsets.US_ASCII).trim();
    }
}
