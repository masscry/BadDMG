package org.baddmg.baddmg;

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
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;

/**
 * Launcher activity. Opens the emulator when a ROM has already been imported,
 * otherwise lets the user pick one through the storage access framework and
 * copies it into app-private storage, where the emulator keeps its .sav files.
 */
public class RomPickerActivity extends Activity {
    public static final String ACTION_PICK_ROM = "org.baddmg.baddmg.PICK_ROM";
    public static final String EXTRA_ROM_PATH = "org.baddmg.baddmg.ROM_PATH";

    private static final String TAG = "BadDMG";
    private static final String PREFS = "baddmg";
    private static final String PREF_ROM_PATH = "rom_path";
    private static final int REQUEST_ROM = 1;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        if (!ACTION_PICK_ROM.equals(getIntent().getAction())) {
            String rom = storedRom(this);
            if (rom != null && new File(rom).isFile()) {
                launchGame(rom);
                return;
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
            Toast.makeText(this, R.string.import_failed, Toast.LENGTH_LONG).show();
            finish();
        }
    }

    private String importRom(Uri uri) throws IOException {
        File dir = new File(getFilesDir(), "roms");
        if (!dir.isDirectory() && !dir.mkdirs()) {
            throw new IOException("failed to create " + dir);
        }
        // A plain path lets the emulator mmap a .sav next to the ROM.
        File rom = new File(dir, safeName(displayName(uri)));
        try (InputStream in = getContentResolver().openInputStream(uri);
             OutputStream out = new FileOutputStream(rom)) {
            if (in == null) {
                throw new IOException("failed to open " + uri);
            }
            byte[] buffer = new byte[64 * 1024];
            int read;
            while ((read = in.read(buffer)) != -1) {
                out.write(buffer, 0, read);
            }
        }
        getSharedPreferences(PREFS, MODE_PRIVATE)
                .edit().putString(PREF_ROM_PATH, rom.getAbsolutePath()).apply();
        return rom.getAbsolutePath();
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
