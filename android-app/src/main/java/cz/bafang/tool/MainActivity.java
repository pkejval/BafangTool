package cz.bafang.tool;

import android.app.Activity;
import android.app.PendingIntent;
import android.content.BroadcastReceiver;
import android.content.res.ColorStateList;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.SharedPreferences;
import android.graphics.Color;
import android.graphics.Typeface;
import android.graphics.drawable.GradientDrawable;
import android.hardware.usb.UsbConstants;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbInterface;
import android.hardware.usb.UsbManager;
import android.os.Build;
import android.os.Bundle;
import android.view.View;
import android.widget.ArrayAdapter;
import android.widget.EditText;
import android.widget.LinearLayout;
import android.widget.ScrollView;
import android.widget.Spinner;
import android.widget.TextView;
import android.widget.Toast;

import com.chaquo.python.PyObject;
import com.chaquo.python.Python;
import com.chaquo.python.android.AndroidPlatform;
import com.google.android.material.button.MaterialButton;
import com.google.android.material.card.MaterialCardView;
import com.google.android.material.textfield.TextInputEditText;
import com.google.android.material.textfield.TextInputLayout;
import com.hoho.android.usbserial.driver.CdcAcmSerialDriver;
import com.hoho.android.usbserial.driver.UsbSerialDriver;
import com.hoho.android.usbserial.driver.UsbSerialPort;
import com.hoho.android.usbserial.driver.UsbSerialProber;

import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import org.json.JSONObject;

public class MainActivity extends Activity {
    private static final String ACTION_USB_PERMISSION = "cz.bafang.tool.USB_PERMISSION";
    private static final int COLOR_PRIMARY = Color.rgb(21, 94, 117);
    private static final int COLOR_PRIMARY_DARK = Color.rgb(15, 61, 76);
    private static final int COLOR_ACCENT = Color.rgb(249, 115, 22);
    private static final int COLOR_BACKGROUND = Color.rgb(244, 247, 248);
    private static final int COLOR_TEXT = Color.rgb(15, 23, 42);
    private static final int COLOR_MUTED = Color.rgb(71, 85, 105);
    private static final int COLOR_OK = Color.rgb(22, 101, 52);
    private static final int COLOR_WARN = Color.rgb(146, 64, 14);

    private final ExecutorService worker = Executors.newSingleThreadExecutor();
    private final List<UsbSerialDriver> drivers = new ArrayList<>();
    private UsbManager usbManager;
    private Spinner portSpinner;
    private ArrayAdapter<String> portAdapter;
    private TextView statusView;
    private TextView statusHintView;
    private TextView logView;
    private TextInputEditText basicEditor;
    private TextInputEditText pedalEditor;
    private TextInputEditText throttleEditor;
    private TextInputEditText rawCommandEditor;
    private TextInputEditText rawWriteEditor;
    private TextInputEditText wheelEditor;
    private TextInputEditText profileNameEditor;
    private UsbSerialPort serialPort;
    private PyObject bafang;

    private final BroadcastReceiver usbReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            if (ACTION_USB_PERMISSION.equals(intent.getAction())) {
                UsbDevice device = intent.getParcelableExtra(UsbManager.EXTRA_DEVICE);
                boolean granted = intent.getBooleanExtra(UsbManager.EXTRA_PERMISSION_GRANTED, false);
                if (granted && device != null) {
                    openSelectedPort();
                } else {
                    appendLog("USB oprávnění zamítnuto");
                }
            } else if (UsbManager.ACTION_USB_DEVICE_ATTACHED.equals(intent.getAction())) {
                scanPorts();
            } else if (UsbManager.ACTION_USB_DEVICE_DETACHED.equals(intent.getAction())) {
                disconnect();
                scanPorts();
            }
        }
    };

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        if (!Python.isStarted()) {
            Python.start(new AndroidPlatform(this));
        }
        usbManager = (UsbManager) getSystemService(Context.USB_SERVICE);
        buildUi();
        registerUsbReceiver();
        scanPorts();
    }

    @Override
    protected void onDestroy() {
        unregisterReceiver(usbReceiver);
        disconnect();
        worker.shutdownNow();
        super.onDestroy();
    }

    private void buildUi() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.LOLLIPOP) {
            getWindow().setStatusBarColor(COLOR_PRIMARY_DARK);
            getWindow().setNavigationBarColor(COLOR_BACKGROUND);
        }

        LinearLayout root = new LinearLayout(this);
        root.setOrientation(LinearLayout.VERTICAL);
        root.setBackgroundColor(COLOR_BACKGROUND);

        root.addView(toolbar());

        ScrollView page = new ScrollView(this);
        page.setFillViewport(true);
        LinearLayout content = new LinearLayout(this);
        content.setOrientation(LinearLayout.VERTICAL);
        content.setPadding(dp(16), dp(16), dp(16), dp(16));
        page.addView(content);

        MaterialCardView connectionCard = card();
        LinearLayout connectionContent = cardContent();
        connectionContent.addView(sectionTitle("Připojení", "USB OTG adaptér USB to SERIAL"));
        statusView = statusPill("Nepřipojeno", COLOR_WARN);
        connectionContent.addView(statusView);
        statusHintView = bodyText("Připojte kabel, vyberte port a povolte USB oprávnění.");
        connectionContent.addView(statusHintView);
        portSpinner = new Spinner(this);
        portAdapter = new ArrayAdapter<>(this, android.R.layout.simple_spinner_item, new ArrayList<>());
        portAdapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        portSpinner.setAdapter(portAdapter);
        LinearLayout.LayoutParams spinnerParams = new LinearLayout.LayoutParams(LinearLayout.LayoutParams.MATCH_PARENT, LinearLayout.LayoutParams.WRAP_CONTENT);
        spinnerParams.setMargins(0, dp(12), 0, dp(8));
        connectionContent.addView(portSpinner, spinnerParams);

        LinearLayout connectionActions = row();
        connectionActions.addView(button("Obnovit", false, view -> scanPorts()));
        connectionActions.addView(button("Připojit", true, view -> connectSelected()));
        connectionActions.addView(button("Odpojit", false, view -> disconnect()));
        connectionContent.addView(connectionActions);
        connectionCard.addView(connectionContent);
        content.addView(connectionCard);

        MaterialCardView actionsCard = card();
        LinearLayout actionsContent = cardContent();
        actionsContent.addView(sectionTitle("Řadič", "Stejné akce jako desktop API"));
        LinearLayout actionRow = row();
        actionRow.addView(button("Načíst vše", true, view -> readAll()));
        actionRow.addView(button("Live data", false, view -> readLiveData()));
        actionRow.addView(button("Chyby", false, view -> readErrors()));
        actionsContent.addView(actionRow);
        LinearLayout actionRow2 = row();
        actionRow2.addView(button("Config", false, view -> callAndLog("Config version", "config_version_json")));
        actionRow2.addView(button("Experimental", false, view -> callAndLog("Experimental", "read_experimental_json")));
        actionRow2.addView(button("Scan", false, view -> callAndLog("Scan commands", "scan_commands_json")));
        actionsContent.addView(actionRow2);
        LinearLayout actionRow3 = row();
        actionRow3.addView(button("Torque calib", false, view -> callAndLog("Torque calibration", "torque_calibration_json")));
        actionRow3.addView(button("Reset", false, view -> callAndLog("Reset", "reset_to_defaults_json")));
        actionsContent.addView(actionRow3);
        wheelEditor = textInput(actionsContent, "Obvod kola (mm)", "2105", false);
        actionsContent.addView(button("Zapsat obvod kola", true, view -> setWheelCircumference()));
        actionsCard.addView(actionsContent);
        content.addView(actionsCard);

        MaterialCardView paramsCard = card();
        LinearLayout paramsContent = cardContent();
        paramsContent.addView(sectionTitle("Parametry", "JSON editace sdílených Python dat"));
        basicEditor = textInput(paramsContent, "Basic", "{}", true);
        LinearLayout basicRow = row();
        basicRow.addView(button("Načíst Basic", false, view -> readSectionToEditor("Basic", "read_basic_json", basicEditor)));
        basicRow.addView(button("Zapsat Basic", true, view -> writeSection("Basic", "write_basic_json", basicEditor)));
        paramsContent.addView(basicRow);
        pedalEditor = textInput(paramsContent, "Pedal", "{}", true);
        LinearLayout pedalRow = row();
        pedalRow.addView(button("Načíst Pedal", false, view -> readSectionToEditor("Pedal", "read_pedal_json", pedalEditor)));
        pedalRow.addView(button("Zapsat Pedal", true, view -> writeSection("Pedal", "write_pedal_json", pedalEditor)));
        paramsContent.addView(pedalRow);
        throttleEditor = textInput(paramsContent, "Throttle", "{}", true);
        LinearLayout throttleRow = row();
        throttleRow.addView(button("Načíst Throttle", false, view -> readSectionToEditor("Throttle", "read_throttle_json", throttleEditor)));
        throttleRow.addView(button("Zapsat Throttle", true, view -> writeSection("Throttle", "write_throttle_json", throttleEditor)));
        paramsContent.addView(throttleRow);
        paramsContent.addView(button("Zapsat vše", true, view -> writeAll()));
        paramsCard.addView(paramsContent);
        content.addView(paramsCard);

        MaterialCardView profilesCard = card();
        LinearLayout profilesContent = cardContent();
        profilesContent.addView(sectionTitle("Profily", "Uložení a aplikace JSON konfigurace"));
        profileNameEditor = textInput(profilesContent, "Název profilu", "M400 default", false);
        LinearLayout profilesRow = row();
        profilesRow.addView(button("Uložit", true, view -> saveProfile()));
        profilesRow.addView(button("Načíst", false, view -> loadProfile()));
        profilesRow.addView(button("Smazat", false, view -> deleteProfile()));
        profilesContent.addView(profilesRow);
        profilesContent.addView(button("Vypsat profily", false, view -> listProfiles()));
        profilesCard.addView(profilesContent);
        content.addView(profilesCard);

        MaterialCardView rawCard = card();
        LinearLayout rawContent = cardContent();
        rawContent.addView(sectionTitle("Raw režim", "Manual command/write a raw bloky"));
        LinearLayout rawReads = row();
        rawReads.addView(button("Raw Basic", false, view -> callAndLog("Raw Basic", "read_raw_basic_json")));
        rawReads.addView(button("Raw Pedal", false, view -> callAndLog("Raw Pedal", "read_raw_pedal_json")));
        rawReads.addView(button("Raw Throttle", false, view -> callAndLog("Raw Throttle", "read_raw_throttle_json")));
        rawContent.addView(rawReads);
        rawCommandEditor = textInput(rawContent, "Manual command hex", "11 51 04 B0 05", false);
        rawContent.addView(button("Odeslat command", true, view -> sendRawCommand()));
        rawWriteEditor = textInput(rawContent, "Manual write hex", "", false);
        rawContent.addView(button("Odeslat write", true, view -> writeCustomRaw()));
        rawCard.addView(rawContent);
        content.addView(rawCard);

        MaterialCardView logCard = card();
        LinearLayout logContent = cardContent();
        logContent.addView(sectionTitle("Komunikační log", "Odpovědi řadiče a raw data"));
        logView = new TextView(this);
        logView.setTextIsSelectable(true);
        logView.setTextColor(COLOR_TEXT);
        logView.setTextSize(13);
        logView.setTypeface(Typeface.MONOSPACE);
        logView.setPadding(dp(12), dp(12), dp(12), dp(12));
        logView.setBackground(rounded(Color.rgb(236, 244, 246), dp(12)));
        logView.setText("Připojte USB OTG adaptér USB to SERIAL a zvolte port.\n");
        logContent.addView(logView, new LinearLayout.LayoutParams(LinearLayout.LayoutParams.MATCH_PARENT, LinearLayout.LayoutParams.WRAP_CONTENT));
        logCard.addView(logContent);
        content.addView(logCard);

        root.addView(page, new LinearLayout.LayoutParams(LinearLayout.LayoutParams.MATCH_PARENT, 0, 1));

        setContentView(root);
    }

    private View toolbar() {
        LinearLayout toolbar = new LinearLayout(this);
        toolbar.setOrientation(LinearLayout.HORIZONTAL);
        toolbar.setGravity(android.view.Gravity.CENTER_VERTICAL);
        toolbar.setPadding(dp(20), dp(18), dp(20), dp(18));
        toolbar.setBackgroundColor(COLOR_PRIMARY_DARK);

        TextView mark = new TextView(this);
        mark.setText("BT");
        mark.setTextColor(Color.WHITE);
        mark.setTextSize(16);
        mark.setTypeface(Typeface.DEFAULT_BOLD);
        mark.setGravity(android.view.Gravity.CENTER);
        mark.setBackground(rounded(COLOR_ACCENT, dp(14)));
        toolbar.addView(mark, new LinearLayout.LayoutParams(dp(44), dp(44)));

        LinearLayout texts = new LinearLayout(this);
        texts.setOrientation(LinearLayout.VERTICAL);
        texts.setPadding(dp(14), 0, 0, 0);
        TextView title = new TextView(this);
        title.setText("BafangTool");
        title.setTextColor(Color.WHITE);
        title.setTextSize(22);
        title.setTypeface(Typeface.DEFAULT_BOLD);
        TextView subtitle = new TextView(this);
        subtitle.setText("USB OTG konfigurace řadiče");
        subtitle.setTextColor(Color.rgb(207, 250, 254));
        subtitle.setTextSize(13);
        texts.addView(title);
        texts.addView(subtitle);
        toolbar.addView(texts, new LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT, 1));
        return toolbar;
    }

    private MaterialCardView card() {
        MaterialCardView card = new MaterialCardView(this);
        card.setCardBackgroundColor(Color.WHITE);
        card.setRadius(dp(20));
        card.setCardElevation(dp(1));
        card.setStrokeWidth(dp(1));
        card.setStrokeColor(Color.rgb(226, 232, 240));
        LinearLayout.LayoutParams params = new LinearLayout.LayoutParams(LinearLayout.LayoutParams.MATCH_PARENT, LinearLayout.LayoutParams.WRAP_CONTENT);
        params.setMargins(0, 0, 0, dp(14));
        card.setLayoutParams(params);
        return card;
    }

    private LinearLayout cardContent() {
        LinearLayout content = new LinearLayout(this);
        content.setOrientation(LinearLayout.VERTICAL);
        content.setPadding(dp(16), dp(16), dp(16), dp(16));
        return content;
    }

    private View sectionTitle(String title, String subtitle) {
        LinearLayout group = new LinearLayout(this);
        group.setOrientation(LinearLayout.VERTICAL);
        TextView titleView = new TextView(this);
        titleView.setText(title);
        titleView.setTextColor(COLOR_TEXT);
        titleView.setTextSize(18);
        titleView.setTypeface(Typeface.DEFAULT_BOLD);
        TextView subtitleView = new TextView(this);
        subtitleView.setText(subtitle);
        subtitleView.setTextColor(COLOR_MUTED);
        subtitleView.setTextSize(13);
        group.addView(titleView);
        group.addView(subtitleView);
        return group;
    }

    private TextView statusPill(String text, int color) {
        TextView pill = new TextView(this);
        pill.setText(text);
        pill.setTextColor(color);
        pill.setTextSize(14);
        pill.setTypeface(Typeface.DEFAULT_BOLD);
        pill.setPadding(dp(12), dp(7), dp(12), dp(7));
        pill.setBackground(rounded(withAlpha(color, 28), dp(18)));
        LinearLayout.LayoutParams params = new LinearLayout.LayoutParams(LinearLayout.LayoutParams.WRAP_CONTENT, LinearLayout.LayoutParams.WRAP_CONTENT);
        params.setMargins(0, dp(14), 0, dp(8));
        pill.setLayoutParams(params);
        return pill;
    }

    private TextView bodyText(String text) {
        TextView view = new TextView(this);
        view.setText(text);
        view.setTextColor(COLOR_MUTED);
        view.setTextSize(14);
        return view;
    }

    private LinearLayout row() {
        LinearLayout row = new LinearLayout(this);
        row.setOrientation(LinearLayout.HORIZONTAL);
        row.setPadding(0, dp(8), 0, 0);
        return row;
    }

    private MaterialButton button(String text, boolean filled, View.OnClickListener listener) {
        MaterialButton button = new MaterialButton(this);
        button.setText(text);
        button.setOnClickListener(listener);
        button.setAllCaps(false);
        button.setCornerRadius(dp(14));
        button.setMinHeight(dp(48));
        if (filled) {
            button.setBackgroundTintList(ColorStateList.valueOf(COLOR_PRIMARY));
            button.setTextColor(Color.WHITE);
        } else {
            button.setBackgroundTintList(ColorStateList.valueOf(Color.TRANSPARENT));
            button.setTextColor(COLOR_PRIMARY);
            button.setStrokeColor(ColorStateList.valueOf(Color.rgb(203, 213, 225)));
            button.setStrokeWidth(dp(1));
        }
        LinearLayout.LayoutParams params = new LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT, 1);
        params.setMargins(dp(3), 0, dp(3), 0);
        button.setLayoutParams(params);
        return button;
    }

    private TextInputEditText textInput(LinearLayout parent, String label, String value, boolean multiline) {
        TextInputLayout layout = new TextInputLayout(this);
        layout.setHint(label);
        layout.setBoxBackgroundMode(TextInputLayout.BOX_BACKGROUND_OUTLINE);
        layout.setBoxStrokeColor(COLOR_PRIMARY);
        layout.setHintTextColor(ColorStateList.valueOf(COLOR_MUTED));
        TextInputEditText input = new TextInputEditText(layout.getContext());
        input.setText(value);
        input.setTextColor(COLOR_TEXT);
        input.setTextSize(13);
        if (multiline) {
            input.setMinLines(5);
            input.setMaxLines(10);
            input.setTypeface(Typeface.MONOSPACE);
            input.setGravity(android.view.Gravity.TOP | android.view.Gravity.START);
        }
        layout.addView(input);
        LinearLayout.LayoutParams params = new LinearLayout.LayoutParams(LinearLayout.LayoutParams.MATCH_PARENT, LinearLayout.LayoutParams.WRAP_CONTENT);
        params.setMargins(0, dp(12), 0, dp(8));
        parent.addView(layout, params);
        return input;
    }

    private void registerUsbReceiver() {
        IntentFilter filter = new IntentFilter(ACTION_USB_PERMISSION);
        filter.addAction(UsbManager.ACTION_USB_DEVICE_ATTACHED);
        filter.addAction(UsbManager.ACTION_USB_DEVICE_DETACHED);
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
            registerReceiver(usbReceiver, filter, Context.RECEIVER_NOT_EXPORTED);
        } else {
            registerReceiver(usbReceiver, filter);
        }
    }

    private void scanPorts() {
        drivers.clear();
        drivers.addAll(UsbSerialProber.getDefaultProber().findAllDrivers(usbManager));
        for (UsbDevice device : usbManager.getDeviceList().values()) {
            if (!hasDriverFor(device) && looksLikeCdcAcm(device)) {
                drivers.add(new CdcAcmSerialDriver(device));
            }
        }
        portAdapter.clear();
        for (UsbSerialDriver driver : drivers) {
            UsbDevice device = driver.getDevice();
            portAdapter.add(device.getDeviceName() + " VID:" + device.getVendorId() + " PID:" + device.getProductId());
        }
        portAdapter.notifyDataSetChanged();
        if (drivers.isEmpty()) {
            updateStatus("Nenalezen USB serial adaptér", "Zkontrolujte OTG redukci a napájení kabelu.", COLOR_WARN);
        } else {
            updateStatus("Porty nalezeny: " + drivers.size(), "Vyberte port a pokračujte připojením k řadiči.", COLOR_PRIMARY);
        }
    }

    private boolean hasDriverFor(UsbDevice device) {
        for (UsbSerialDriver driver : drivers) {
            if (driver.getDevice().getDeviceId() == device.getDeviceId()) {
                return true;
            }
        }
        return false;
    }

    private boolean looksLikeCdcAcm(UsbDevice device) {
        if (device.getDeviceClass() == UsbConstants.USB_CLASS_COMM) {
            return true;
        }
        for (int i = 0; i < device.getInterfaceCount(); i++) {
            UsbInterface usbInterface = device.getInterface(i);
            if (usbInterface.getInterfaceClass() == UsbConstants.USB_CLASS_COMM
                    || usbInterface.getInterfaceClass() == UsbConstants.USB_CLASS_CDC_DATA) {
                return true;
            }
        }
        return false;
    }

    private void connectSelected() {
        if (drivers.isEmpty()) {
            toast("Nenalezen žádný USB serial adaptér");
            scanPorts();
            return;
        }
        UsbSerialDriver driver = drivers.get(portSpinner.getSelectedItemPosition());
        UsbDevice device = driver.getDevice();
        if (!usbManager.hasPermission(device)) {
            int flags = Build.VERSION.SDK_INT >= Build.VERSION_CODES.S ? PendingIntent.FLAG_MUTABLE : 0;
            PendingIntent intent = PendingIntent.getBroadcast(this, 0, new Intent(ACTION_USB_PERMISSION), flags);
            usbManager.requestPermission(device, intent);
            appendLog("Vyžádáno USB oprávnění");
            return;
        }
        openSelectedPort();
    }

    private void openSelectedPort() {
        if (drivers.isEmpty()) {
            scanPorts();
            return;
        }
        UsbSerialDriver driver = drivers.get(portSpinner.getSelectedItemPosition());
        worker.execute(() -> {
            disconnectPortOnly();
            UsbDeviceConnection connection = usbManager.openDevice(driver.getDevice());
            if (connection == null) {
                appendLog("USB zařízení nelze otevřít");
                return;
            }
            try {
                serialPort = driver.getPorts().get(0);
                serialPort.open(connection);
                serialPort.setParameters(BafangProtocol.BAUD_RATE, 8, UsbSerialPort.STOPBITS_1, UsbSerialPort.PARITY_NONE);
                serialPort.purgeHwBuffers(true, true);
                try {
                    serialPort.setDTR(false);
                    serialPort.setRTS(false);
                } catch (UnsupportedOperationException ignored) {
                }
                PyObject bridgeClass = Python.getInstance().getModule("android_bridge").get("AndroidBafangController");
                bafang = bridgeClass.call(new AndroidSerialPortBridge(serialPort));
                boolean connected = bafang.callAttr("connect").toBoolean();
                if (!connected) {
                    disconnectPortOnly();
                    appendLog("Bafang handshake selhal");
                    runOnUiThread(() -> updateStatus("Handshake selhal", "Řadič neodpověděl na Bafang inicializaci.", COLOR_WARN));
                    return;
                }
                String status = bafang.callAttr("status_json").toString();
                appendLog("Připojeno\n" + status);
                runOnUiThread(() -> updateStatus("Připojeno", "Komunikace běží přes sdílený Python protokol.", COLOR_OK));
            } catch (Exception e) {
                disconnectPortOnly();
                appendLog("Chyba připojení: " + e.getMessage());
                runOnUiThread(() -> updateStatus("Nepřipojeno", "Chyba připojení: " + e.getMessage(), COLOR_WARN));
            }
        });
    }

    private void readAll() {
        withConnection(() -> {
            String json = bafang.callAttr("read_all_json").toString();
            appendLog("Načteno vše\n" + json);
            runOnUiThread(() -> populateEditorsFromReadAll(json));
        });
    }

    private void readLiveData() {
        callAndLog("Live data", "live_data_json");
    }

    private void readErrors() {
        callAndLog("Errors", "errors_json");
    }

    private void readSectionToEditor(String label, String method, EditText editor) {
        withConnection(() -> {
            String json = bafang.callAttr(method).toString();
            runOnUiThread(() -> editor.setText(json));
            appendLog(label + " načteno\n" + json);
        });
    }

    private void writeSection(String label, String method, EditText editor) {
        withConnection(() -> appendLog(label + " write\n" + bafang.callAttr(method, editor.getText().toString()).toString()));
    }

    private void writeAll() {
        withConnection(() -> appendLog("Write all\n" + bafang.callAttr(
                "write_all_json",
                basicEditor.getText().toString(),
                pedalEditor.getText().toString(),
                throttleEditor.getText().toString()).toString()));
    }

    private void callAndLog(String label, String method) {
        withConnection(() -> appendLog(label + "\n" + bafang.callAttr(method).toString()));
    }

    private void sendRawCommand() {
        withConnection(() -> appendLog("Manual command\n" + bafang.callAttr("send_raw_command_json", rawCommandEditor.getText().toString()).toString()));
    }

    private void writeCustomRaw() {
        withConnection(() -> appendLog("Manual write\n" + bafang.callAttr("write_custom_raw_json", rawWriteEditor.getText().toString()).toString()));
    }

    private void setWheelCircumference() {
        withConnection(() -> appendLog("Wheel circumference\n" + bafang.callAttr("set_wheel_circumference_json", wheelEditor.getText().toString()).toString()));
    }

    private void saveProfile() {
        try {
            String name = profileName();
            JSONObject profile = new JSONObject();
            profile.put("basic", new JSONObject(basicEditor.getText().toString()));
            profile.put("pedal", new JSONObject(pedalEditor.getText().toString()));
            profile.put("throttle", new JSONObject(throttleEditor.getText().toString()));
            profiles().edit().putString(name, profile.toString(2)).apply();
            appendLog("Profil uložen: " + name);
        } catch (Exception e) {
            appendLog("Profil nelze uložit: " + e.getMessage());
        }
    }

    private void loadProfile() {
        String name = profileName();
        String value = profiles().getString(name, null);
        if (value == null) {
            appendLog("Profil nenalezen: " + name);
            return;
        }
        try {
            JSONObject profile = new JSONObject(value);
            basicEditor.setText(profile.getJSONObject("basic").toString(2));
            pedalEditor.setText(profile.getJSONObject("pedal").toString(2));
            throttleEditor.setText(profile.getJSONObject("throttle").toString(2));
            appendLog("Profil načten: " + name);
        } catch (Exception e) {
            appendLog("Profil nelze načíst: " + e.getMessage());
        }
    }

    private void deleteProfile() {
        String name = profileName();
        profiles().edit().remove(name).apply();
        appendLog("Profil smazán: " + name);
    }

    private void listProfiles() {
        appendLog("Profily\n" + profiles().getAll().keySet().toString());
    }

    private SharedPreferences profiles() {
        return getSharedPreferences("profiles", MODE_PRIVATE);
    }

    private String profileName() {
        String name = profileNameEditor.getText() == null ? "" : profileNameEditor.getText().toString().trim();
        return name.isEmpty() ? "Default" : name;
    }

    private void populateEditorsFromReadAll(String json) {
        try {
            JSONObject root = new JSONObject(json);
            if (root.has("basic")) {
                basicEditor.setText(root.getJSONObject("basic").toString(2));
            }
            if (root.has("pedal")) {
                pedalEditor.setText(root.getJSONObject("pedal").toString(2));
            }
            if (root.has("throttle")) {
                throttleEditor.setText(root.getJSONObject("throttle").toString(2));
            }
        } catch (Exception e) {
            appendLog("JSON nelze rozdělit do editorů: " + e.getMessage());
        }
    }

    private void withConnection(SerialAction action) {
        if (bafang == null || serialPort == null) {
            toast("Nejdřív se připojte k řadiči");
            return;
        }
        worker.execute(() -> {
            try {
                action.run();
            } catch (Exception e) {
                appendLog("Komunikační chyba: " + e.getMessage());
            }
        });
    }

    private void disconnect() {
        worker.execute(() -> {
            disconnectPortOnly();
            runOnUiThread(() -> updateStatus("Nepřipojeno", "Připojte kabel a vyberte USB serial port.", COLOR_WARN));
            appendLog("Odpojeno");
        });
    }

    private void updateStatus(String status, String hint, int color) {
        statusView.setText(status);
        statusView.setTextColor(color);
        statusView.setBackground(rounded(withAlpha(color, 28), dp(18)));
        statusHintView.setText(hint);
    }

    private void disconnectPortOnly() {
        if (bafang != null) {
            try {
                bafang.callAttr("disconnect");
            } catch (Exception ignored) {
            }
        }
        bafang = null;
        if (serialPort != null) {
            try {
                serialPort.close();
            } catch (IOException ignored) {
            }
        }
        serialPort = null;
    }

    private void appendLog(String message) {
        String stamp = new SimpleDateFormat("HH:mm:ss", Locale.US).format(new Date());
        runOnUiThread(() -> logView.append("\n[" + stamp + "] " + message + "\n"));
    }

    private void toast(String message) {
        runOnUiThread(() -> Toast.makeText(this, message, Toast.LENGTH_SHORT).show());
    }

    private int dp(int value) {
        return Math.round(value * getResources().getDisplayMetrics().density);
    }

    private GradientDrawable rounded(int color, int radius) {
        GradientDrawable drawable = new GradientDrawable();
        drawable.setColor(color);
        drawable.setCornerRadius(radius);
        return drawable;
    }

    private int withAlpha(int color, int alpha) {
        return Color.argb(alpha, Color.red(color), Color.green(color), Color.blue(color));
    }

    private interface SerialAction {
        void run() throws Exception;
    }
}
