package space.deci.baddmg;

import android.app.Activity;
import android.content.Context;
import android.content.Intent;
import android.content.SharedPreferences;
import android.database.Cursor;
import android.net.Uri;
import android.os.Bundle;
import android.provider.OpenableColumns;
import android.util.Log;
import android.view.Gravity;
import android.widget.TextView;
import android.widget.Toast;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.Locale;
import java.util.zip.ZipEntry;
import java.util.zip.ZipInputStream;

/**
 * Launcher activity. Opens the emulator when a ROM has already been imported,
 * otherwise lets the user pick one through the storage access framework and
 * copies it into app-private storage, where the emulator keeps its .sav files.
 * ZIP archives are unpacked, the emulator only reads plain ROM images.
 */
public class RomPickerActivity extends Activity {
    public static final String ACTION_PICK_ROM = "space.deci.baddmg.PICK_ROM";
    public static final String EXTRA_ROM_PATH = "space.deci.baddmg.ROM_PATH";

    private static final String TAG = "BadDMG";
    private static final String PREFS = "baddmg";
    private static final String PREF_ROM_PATH = "rom_path";
    private static final int REQUEST_ROM = 1;

    // Cartridge types the emulator accepts, see src/bdmg/cartridge.hh.
    private static final int[] CARTRIDGE_TYPES = {
        0x00, 0x01, 0x02, 0x03, 0x05, 0x06, 0x08, 0x09, 0x0b, 0x0c, 0x0d,
        0x0f, 0x10, 0x11, 0x12, 0x13, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e,
        0x20, 0x22, 0xfc, 0xfd, 0xfe, 0xff
    };

    /** Import failure the user should see, unlike the detail in the message. */
    private static class RomError extends IOException {
        private final int message;

        RomError(int message, String detail) {
            super(detail);
            this.message = message;
        }
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        if (!ACTION_PICK_ROM.equals(getIntent().getAction())) {
            String rom = storedRom(this);
            if (rom != null && new File(rom).isFile()) {
                try {
                    launchGame(usableRom(rom));
                    return;
                } catch (IOException e) {
                    Log.e(TAG, "failed to use the stored ROM", e);
                }
            }
        }

        // Only visible behind the system picker.
        TextView message = new TextView(this);
        message.setText(R.string.select_rom);
        message.setGravity(Gravity.CENTER);
        setContentView(message);

        Intent intent = new Intent(Intent.ACTION_OPEN_DOCUMENT);
        intent.addCategory(Intent.CATEGORY_OPENABLE);
        intent.setType("*/*");
        startActivityForResult(intent, REQUEST_ROM);
    }

    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        super.onActivityResult(requestCode, resultCode, data);
        if (requestCode != REQUEST_ROM) {
            return;
        }
        if (resultCode != RESULT_OK || data == null || data.getData() == null) {
            finish();
            return;
        }
        try {
            launchGame(importRom(data.getData()));
        } catch (IOException e) {
            Log.e(TAG, "failed to import ROM", e);
            int message = e instanceof RomError
                ? ((RomError) e).message
                : R.string.import_failed;
            Toast.makeText(this, message, Toast.LENGTH_LONG).show();
            finish();
        }
    }

    private String importRom(Uri uri) throws IOException {
        String name = displayName(uri);
        File rom;
        try (InputStream in = open(uri)) {
            if (isArchive(name)) {
                rom = unpackArchive(in, name);
            } else {
                // A plain path lets the emulator mmap a .sav next to the ROM.
                rom = new File(romDir(), safeName(name));
                try (OutputStream out = new FileOutputStream(rom)) {
                    copy(in, out);
                }
            }
        }
        checkRom(rom);
        remember(rom);
        return rom.getAbsolutePath();
    }

    /**
     * Returns the ROM to run for an already imported file. Archives left over
     * from an older version are unpacked now.
     */
    private String usableRom(String path) throws IOException {
        File rom = new File(path);
        if (!isArchive(path)) {
            checkRom(rom);
            return path;
        }
        try (InputStream in = new FileInputStream(rom)) {
            File unpacked = unpackArchive(in, rom.getName());
            checkRom(unpacked);
            remember(unpacked);
            return unpacked.getAbsolutePath();
        }
    }

    /** Extracts the first ROM image from a ZIP archive. */
    private File unpackArchive(InputStream in, String archive) throws IOException {
        ZipInputStream zip = new ZipInputStream(in);
        ZipEntry entry;
        while ((entry = zip.getNextEntry()) != null) {
            if (entry.isDirectory()) {
                continue;
            }
            String name = baseName(entry.getName());
            if (!name.toLowerCase(Locale.ROOT).endsWith(".gb")
                && !name.toLowerCase(Locale.ROOT).endsWith(".gbc")) {
                continue;
            }
            File rom = new File(romDir(), safeName(name));
            try (OutputStream out = new FileOutputStream(rom)) {
                copy(zip, out);
            }
            return rom;
        }
        throw new RomError(R.string.no_rom_in_archive,
                           "no .gb or .gbc file in " + archive);
    }

    /** Rejects files the emulator would not be able to boot. */
    private static void checkRom(File rom) throws IOException {
        if (rom.length() < 0x150) {
            throw new RomError(R.string.not_a_rom, "too small: " + rom);
        }
        int type;
        try (InputStream in = new FileInputStream(rom)) {
            in.skip(0x147);
            type = in.read();
        }
        for (int known : CARTRIDGE_TYPES) {
            if (known == type) {
                return;
            }
        }
        throw new RomError(R.string.not_a_rom,
                           "unknown cartridge type " + type + ": " + rom);
    }

    private InputStream open(Uri uri) throws IOException {
        InputStream in = getContentResolver().openInputStream(uri);
        if (in == null) {
            throw new IOException("failed to open " + uri);
        }
        return in;
    }

    private static void copy(InputStream in, OutputStream out) throws IOException {
        byte[] buffer = new byte[64 * 1024];
        int read;
        while ((read = in.read(buffer)) != -1) {
            out.write(buffer, 0, read);
        }
    }

    private File romDir() throws IOException {
        File dir = new File(getFilesDir(), "roms");
        if (!dir.isDirectory() && !dir.mkdirs()) {
            throw new IOException("failed to create " + dir);
        }
        return dir;
    }

    private void remember(File rom) {
        getSharedPreferences(PREFS, MODE_PRIVATE)
                .edit().putString(PREF_ROM_PATH, rom.getAbsolutePath()).apply();
    }

    private String displayName(Uri uri) {
        try (Cursor cursor = getContentResolver().query(
                uri, new String[] {OpenableColumns.DISPLAY_NAME}, null, null, null)) {
            if (cursor != null && cursor.moveToFirst()) {
                String name = cursor.getString(0);
                if (name != null && !name.isEmpty()) {
                    return name;
                }
            }
        }
        String name = uri.getLastPathSegment();
        return name != null && !name.isEmpty() ? name : "game.gb";
    }

    private static boolean isArchive(String name) {
        return name.toLowerCase(Locale.ROOT).endsWith(".zip");
    }

    private static String baseName(String path) {
        int slash = path.lastIndexOf('/');
        return slash >= 0 ? path.substring(slash + 1) : path;
    }

    private static String safeName(String name) {
        String safe = name.replace('/', '_').replace('\\', '_');
        return safe.equals(".") || safe.equals("..") ? "game.gb" : safe;
    }

    static String storedRom(Context context) {
        return context.getSharedPreferences(PREFS, MODE_PRIVATE)
                .getString(PREF_ROM_PATH, null);
    }

    private void launchGame(String path) {
        Intent intent = new Intent(this, MainActivity.class);
        intent.putExtra(EXTRA_ROM_PATH, path);
        startActivity(intent);
        finish();
    }
}
