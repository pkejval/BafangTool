package cz.bafang.tool;

import com.hoho.android.usbserial.driver.UsbSerialPort;

import java.io.IOException;

final class AndroidSerialPortBridge {
    private final UsbSerialPort port;
    private boolean open = true;

    AndroidSerialPortBridge(UsbSerialPort port) {
        this.port = port;
    }

    public void writeBytes(byte[] data) throws IOException {
        port.write(data, 2000);
    }

    public byte[] readBytes(int size) throws IOException {
        byte[] buffer = new byte[size];
        int length = port.read(buffer, 80);
        if (length <= 0) {
            return new byte[0];
        }
        byte[] out = new byte[length];
        System.arraycopy(buffer, 0, out, 0, length);
        return out;
    }

    public void purgeInput() throws IOException {
        port.purgeHwBuffers(true, false);
    }

    public void closePort() throws IOException {
        if (open) {
            open = false;
            port.close();
        }
    }
}
