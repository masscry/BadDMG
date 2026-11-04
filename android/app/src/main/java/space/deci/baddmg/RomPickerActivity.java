package space.deci.baddmg;

import android.annotation.TargetApi;
import android.app.Activity;
import android.app.AlertDialog;
import android.content.ContentResolver;
import android.content.Context;
import android.content.Intent;
import android.content.SharedPreferences;
import android.content.pm.ShortcutInfo;
import android.content.pm.ShortcutManager;
import android.database.Cursor;
import android.graphics.drawable.Icon;
import android.net.Uri;
import android.os.Build;
import android.os.Bundle;
import android.provider.OpenableColumns;
import android.util.Log;
import android.util.TypedValue;
import android.view.Gravity;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.LinearLayout;
import android.widget.ListView;
import android.widget.TextView;
import android.widget.Toast;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.nio.charset.StandardCharsets;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Locale;
import java.util.Set;
import java.util.zip.ZipEntry;
import java.util.zip.ZipInputStream;
import java.util.zip.ZipOutputStream;

/**
 * Launcher activity and ROM library. The list is built from the ROMs in
 * app-private storage, so imported ROMs stay available and keep their saves.
 * Tapping one starts the emulator; Back returns to this list.
 */
public class RomPickerActivity extends Activity {
    public static final String ACTION_PICK_ROM = "space.deci.baddmg.PICK_ROM";
    public static final String EXTRA_ROM_PATH = "space.deci.baddmg.ROM_PATH";
    public static final String EXTRA_PLAY_ROM = "space.deci.baddmg.PLAY_ROM";

    private static final String TAG = "BadDMG";
    private static final String PREFS = "baddmg";
    private static final String PREF_ROM_PATH = "rom_path";
    private static final int REQUEST_ROM = 1;
    private static final int REQUEST_EXPORT_SAVE = 2;
    private static final int REQUEST_IMPORT_SAVE = 3;

    // Cartridge types the emulator accepts, see src/bdmg/cartridge.hh.
    private static final int[] CARTRIDGE_TYPES = {
        0x00, 0x01, 0x02, 0x03, 0x05, 0x06, 0x08, 0x09, 0x0b, 0x0c, 0x0d,
        0x0f, 0x10, 0x11, 0x12, 0x13, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e,
        0x20, 0x22, 0xfc, 0xfd, 0xfe, 0xff
    };

    private final List<File> _roms = new ArrayList<>();
    private ArrayAdapter<String> _adapter;
    private ListView _list;
    private TextView _empty;
    private File _pendingSaveRom;

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
        buildUi();
        if (playRequested(getIntent())) {
            return;
        }
        if (_roms.isEmpty() || ACTION_PICK_ROM.equals(getIntent().getAction())) {
            pickRom();
        }
    }

    @Override
    protected void onNewIntent(Intent intent) {
        super.onNewIntent(intent);
        setIntent(intent);
        refreshRoms();
        if (playRequested(intent)) {
            return;
        }
        if (ACTION_PICK_ROM.equals(intent.getAction())) {
            pickRom();
        }
    }

    @Override
    protected void onResume() {
        super.onResume();
        refreshRoms();
    }

    private void buildUi() {
        LinearLayout root = new LinearLayout(this);
        root.setOrientation(LinearLayout.VERTICAL);
        root.setFitsSystemWindows(true);

        TextView title = new TextView(this);
        title.setText(R.string.app_name);
        title.setGravity(Gravity.CENTER);
        title.setTextSize(TypedValue.COMPLEX_UNIT_SP, 24);
        int padding = (int) (24 * getResources().getDisplayMetrics().density);
        title.setPadding(0, padding, 0, padding);
        root.addView(title);

        _empty = new TextView(this);
        _empty.setText(R.string.select_rom);
        _empty.setGravity(Gravity.CENTER);
        root.addView(_empty, new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT, 0, 1));

        _adapter = new ArrayAdapter<>(this, android.R.layout.simple_list_item_1,
                                      new ArrayList<>());
        _list = new ListView(this);
        _list.setAdapter(_adapter);
        _list.setOnItemClickListener((parent, view, position, id) -> {
            if (position >= 0 && position < _roms.size()) {
                launchGame(_roms.get(position).getAbsolutePath());
            }
        });
        _list.setOnItemLongClickListener((parent, view, position, id) -> {
            if (position >= 0 && position < _roms.size()) {
                showRomActions(_roms.get(position));
            }
            return true;
        });
        root.addView(_list, new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT, 0, 1));

        Button add = new Button(this);
        add.setText(R.string.add_rom);
        add.setOnClickListener(view -> pickRom());
        root.addView(add, new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT,
                ViewGroup.LayoutParams.WRAP_CONTENT));

        setContentView(root);
        refreshRoms();
    }

    private void refreshRoms() {
        _roms.clear();
        File dir = romDirOrNull();
        File[] files = dir != null ? dir.listFiles() : null;
        if (files != null) {
            for (File file : files) {
                if (isRom(file)) {
                    _roms.add(file);
                }
            }
        }
        _roms.sort(Comparator.comparing(File::getName, String.CASE_INSENSITIVE_ORDER));
        Set<String> hashes = new HashSet<>();
        for (Iterator<File> it = _roms.iterator(); it.hasNext();) {
            File rom = it.next();
            String hash = hashFor(rom);
            if (hash != null && !hashes.add(hash)) {
                it.remove();
            }
        }
        _adapter.clear();
        for (File rom : _roms) {
            _adapter.add(rom.getName());
        }
        _list.setVisibility(_roms.isEmpty() ? View.GONE : View.VISIBLE);
        _empty.setVisibility(_roms.isEmpty() ? View.VISIBLE : View.GONE);
    }

    private static boolean isRom(File file) {
        if (!file.isFile()) {
            return false;
        }
        String name = file.getName().toLowerCase(Locale.ROOT);
        if (name.endsWith(".sav") || name.endsWith(".rtc") || name.endsWith(".tmp")
                || name.endsWith(".zip") || name.endsWith(".sha256")) {
            return false;
        }
        try {
            checkRom(file);
            return true;
        } catch (IOException e) {
            return false;
        }
    }

    private File romDirOrNull() {
        try {
            return romDir();
        } catch (IOException e) {
            Log.e(TAG, "failed to access the ROM directory", e);
            return null;
        }
    }

    private void pickRom() {
        Intent intent = new Intent(Intent.ACTION_OPEN_DOCUMENT);
        intent.addCategory(Intent.CATEGORY_OPENABLE);
        intent.setType("*/*");
        startActivityForResult(intent, REQUEST_ROM);
    }

    private boolean playRequested(Intent intent) {
        String path = intent.getStringExtra(EXTRA_PLAY_ROM);
        if (path == null) {
            return false;
        }
        intent.removeExtra(EXTRA_PLAY_ROM);
        if (new File(path).isFile()) {
            launchGame(path);
            return true;
        }
        return false;
    }

    private void showRomActions(File rom) {
        String[] actions = {
            getString(R.string.add_to_home),
            getString(R.string.export_save),
            getString(R.string.import_save),
            getString(R.string.delete_rom),
        };
        new AlertDialog.Builder(this)
            .setTitle(rom.getName())
            .setItems(actions, (dialog, which) -> {
                if (which == 0) {
                    pinShortcut(rom);
                } else if (which == 1) {
                    exportSave(rom);
                } else if (which == 2) {
                    importSave(rom);
                } else if (which == 3) {
                    confirmDelete(rom);
                }
            })
            .show();
    }

    private void confirmDelete(File rom) {
        if (MainActivity.isRunning()) {
            Toast.makeText(this, R.string.close_game_first, Toast.LENGTH_SHORT).show();
            return;
        }
        new AlertDialog.Builder(this)
            .setTitle(rom.getName())
            .setMessage(R.string.delete_rom_confirm)
            .setPositiveButton(R.string.delete_rom,
                               (dialog, which) -> deleteRom(rom, false))
            .setNeutralButton(R.string.delete_rom_and_save,
                              (dialog, which) -> deleteRom(rom, true))
            .setNegativeButton(android.R.string.cancel, null)
            .show();
    }

    private void deleteRom(File rom, boolean deleteSaves) {
        String hash = hashFor(rom);
        File dir = romDirOrNull();
        List<File> doomed = new ArrayList<>();
        File[] files = dir != null ? dir.listFiles() : null;
        if (files != null) {
            for (File file : files) {
                if (!isRom(file)) {
                    continue;
                }
                if (file.equals(rom) || (hash != null && hash.equals(hashFor(file)))) {
                    doomed.add(file);
                }
            }
        }
        if (!doomed.contains(rom)) {
            doomed.add(rom);
        }

        boolean ok = true;
        for (File file : doomed) {
            hashSidecar(file).delete();
            if (!file.delete()) {
                ok = false;
            }
            if (deleteSaves) {
                if (hash != null) {
                    deleteIfExists(new File(file.getParentFile(), hash + ".sav"));
                    deleteIfExists(new File(file.getParentFile(), hash + ".rtc"));
                }
                deleteIfExists(oldSave(file, ".sav"));
                deleteIfExists(oldSave(file, ".rtc"));
            }
        }
        disableShortcuts(doomed);
        if (!ok) {
            Toast.makeText(this, R.string.delete_failed, Toast.LENGTH_LONG).show();
        }
        refreshRoms();
    }

    private static boolean deleteIfExists(File file) {
        return !file.exists() || file.delete();
    }

    private static File oldSave(File rom, String extension) {
        String name = rom.getName();
        int dot = name.lastIndexOf('.');
        String stem = dot > 0 ? name.substring(0, dot) : name;
        return new File(rom.getParentFile(), stem + extension);
    }

    private static String shortcutId(File rom) {
        return "rom-" + Integer.toHexString(rom.getAbsolutePath().hashCode());
    }

    private void disableShortcuts(List<File> roms) {
        if (Build.VERSION.SDK_INT < Build.VERSION_CODES.O || roms.isEmpty()) {
            return;
        }
        disableShortcutsApi26(roms);
    }

    @TargetApi(Build.VERSION_CODES.O)
    private void disableShortcutsApi26(List<File> roms) {
        ShortcutManager manager = getSystemService(ShortcutManager.class);
        if (manager == null) {
            return;
        }
        List<String> ids = new ArrayList<>();
        for (File rom : roms) {
            ids.add(shortcutId(rom));
        }
        try {
            manager.disableShortcuts(ids);
        } catch (RuntimeException e) {
            Log.w(TAG, "failed to disable shortcuts", e);
        }
    }

    private void pinShortcut(File rom) {
        if (Build.VERSION.SDK_INT < Build.VERSION_CODES.O) {
            Toast.makeText(this, R.string.shortcut_unsupported, Toast.LENGTH_SHORT).show();
            return;
        }
        pinShortcutApi26(rom);
    }

    @TargetApi(Build.VERSION_CODES.O)
    private void pinShortcutApi26(File rom) {
        ShortcutManager manager = getSystemService(ShortcutManager.class);
        if (manager == null) {
            Toast.makeText(this, R.string.shortcut_unsupported, Toast.LENGTH_SHORT).show();
            return;
        }
        String id = shortcutId(rom);
        Intent intent = new Intent(this, RomPickerActivity.class)
            .setAction(Intent.ACTION_VIEW)
            .putExtra(EXTRA_PLAY_ROM, rom.getAbsolutePath());
        ShortcutInfo shortcut = new ShortcutInfo.Builder(this, id)
            .setShortLabel(rom.getName())
            .setLongLabel(getString(R.string.play_rom, rom.getName()))
            .setIcon(Icon.createWithResource(this, R.mipmap.ic_launcher))
            .setIntent(intent)
            .build();
        if (!manager.requestPinShortcut(shortcut, null)) {
            Toast.makeText(this, R.string.shortcut_unsupported, Toast.LENGTH_SHORT).show();
        }
    }

    private void exportSave(File rom) {
        File sav = savePath(rom, ".sav");
        File rtc = savePath(rom, ".rtc");
        if (sav == null || (!sav.isFile() && (rtc == null || !rtc.isFile()))) {
            Toast.makeText(this, R.string.no_save, Toast.LENGTH_SHORT).show();
            return;
        }
        _pendingSaveRom = rom;
        Intent intent = new Intent(Intent.ACTION_CREATE_DOCUMENT)
            .addCategory(Intent.CATEGORY_OPENABLE)
            .setType("application/zip")
            .putExtra(Intent.EXTRA_TITLE, saveName(rom) + "-save.zip");
        startActivityForResult(intent, REQUEST_EXPORT_SAVE);
    }

    private void exportSaveTo(Uri uri, File rom) {
        File sav = savePath(rom, ".sav");
        File rtc = savePath(rom, ".rtc");
        try {
            checkContentUri(uri);
            try (OutputStream raw = getContentResolver().openOutputStream(uri)) {
                if (raw == null) {
                    throw new IOException("failed to open " + uri);
                }
                try (ZipOutputStream zip = new ZipOutputStream(raw)) {
                    if (sav != null && sav.isFile()) {
                        addZipEntry(zip, sav, saveName(rom) + ".sav");
                    }
                    if (rtc != null && rtc.isFile()) {
                        addZipEntry(zip, rtc, saveName(rom) + ".rtc");
                    }
                }
            }
            Toast.makeText(this, R.string.save_exported, Toast.LENGTH_SHORT).show();
        } catch (IOException e) {
            Log.e(TAG, "failed to export save", e);
            Toast.makeText(this, R.string.save_export_failed, Toast.LENGTH_LONG).show();
        }
    }

    private static void addZipEntry(ZipOutputStream zip, File file, String name)
            throws IOException {
        zip.putNextEntry(new ZipEntry(name));
        try (InputStream in = new FileInputStream(file)) {
            copy(in, zip);
        }
        zip.closeEntry();
    }

    private void importSave(File rom) {
        if (MainActivity.isRunning()) {
            Toast.makeText(this, R.string.close_game_first, Toast.LENGTH_SHORT).show();
            return;
        }
        if (savePath(rom, ".sav") == null) {
            Toast.makeText(this, R.string.save_import_failed, Toast.LENGTH_LONG).show();
            return;
        }
        _pendingSaveRom = rom;
        new AlertDialog.Builder(this)
            .setMessage(R.string.import_save_warning)
            .setPositiveButton(android.R.string.ok, (dialog, which) -> {
                Intent intent = new Intent(Intent.ACTION_OPEN_DOCUMENT)
                    .addCategory(Intent.CATEGORY_OPENABLE)
                    .setType("*/*");
                startActivityForResult(intent, REQUEST_IMPORT_SAVE);
            })
            .setNegativeButton(android.R.string.cancel,
                               (dialog, which) -> _pendingSaveRom = null)
            .show();
    }

    private void importSaveFrom(Uri uri, File rom) {
        File sav = savePath(rom, ".sav");
        File rtc = savePath(rom, ".rtc");
        if (sav == null || rtc == null) {
            Toast.makeText(this, R.string.save_import_failed, Toast.LENGTH_LONG).show();
            return;
        }
        try (InputStream in = open(uri)) {
            String name = displayName(uri);
            if (isArchive(name)) {
                importSaveZip(in, sav, rtc);
            } else if (name.toLowerCase(Locale.ROOT).endsWith(".sav")) {
                replaceFile(in, sav);
            } else if (name.toLowerCase(Locale.ROOT).endsWith(".rtc")) {
                replaceFile(in, rtc);
            } else {
                throw new IOException("unsupported save file " + name);
            }
            Toast.makeText(this, R.string.save_imported, Toast.LENGTH_SHORT).show();
        } catch (IOException e) {
            Log.e(TAG, "failed to import save", e);
            Toast.makeText(this, R.string.save_import_failed, Toast.LENGTH_LONG).show();
        }
    }

    private static void importSaveZip(InputStream in, File sav, File rtc) throws IOException {
        ZipInputStream zip = new ZipInputStream(in);
        boolean found = false;
        ZipEntry entry;
        while ((entry = zip.getNextEntry()) != null) {
            if (entry.isDirectory()) {
                continue;
            }
            String name = baseName(entry.getName()).toLowerCase(Locale.ROOT);
            if (name.endsWith(".sav")) {
                replaceFile(zip, sav);
                found = true;
            } else if (name.endsWith(".rtc")) {
                replaceFile(zip, rtc);
                found = true;
            }
        }
        if (!found) {
            throw new IOException("no save files in archive");
        }
    }

    private static void replaceFile(InputStream in, File target) throws IOException {
        File temp = new File(target.getParentFile(), target.getName() + ".tmp");
        try {
            try (OutputStream out = new FileOutputStream(temp)) {
                copy(in, out);
            }
            if (!temp.renameTo(target)) {
                throw new IOException("failed to replace " + target);
            }
        } catch (IOException e) {
            temp.delete();
            throw e;
        }
    }

    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        super.onActivityResult(requestCode, resultCode, data);
        if (requestCode == REQUEST_EXPORT_SAVE) {
            if (resultCode == RESULT_OK && data != null && data.getData() != null
                    && _pendingSaveRom != null) {
                exportSaveTo(data.getData(), _pendingSaveRom);
            }
            _pendingSaveRom = null;
            return;
        }
        if (requestCode == REQUEST_IMPORT_SAVE) {
            if (resultCode == RESULT_OK && data != null && data.getData() != null
                    && _pendingSaveRom != null) {
                importSaveFrom(data.getData(), _pendingSaveRom);
            }
            _pendingSaveRom = null;
            return;
        }
        if (requestCode != REQUEST_ROM) {
            return;
        }
        if (resultCode != RESULT_OK || data == null || data.getData() == null) {
            refreshRoms();
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
            refreshRoms();
        }
    }

    private String importRom(Uri uri) throws IOException {
        String name = displayName(uri);
        File rom;
        try (InputStream in = open(uri)) {
            rom = isArchive(name) ? unpackArchive(in, name) : store(in, name);
        }
        remember(rom);
        return rom.getAbsolutePath();
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
            return store(zip, name);
        }
        throw new RomError(R.string.no_rom_in_archive,
                           "no .gb or .gbc file in " + archive);
    }

    /**
     * Copies to a temporary file, validates it, then replaces the destination.
     * A failed import must not clobber an already imported ROM, and identical
     * contents reuse the existing file and its saves.
     */
    private File store(InputStream in, String name) throws IOException {
        File target = new File(romDir(), safeName(name));
        File temp = new File(romDir(), safeName(name) + ".tmp");
        MessageDigest digest;
        try {
            digest = MessageDigest.getInstance("SHA-256");
        } catch (NoSuchAlgorithmException e) {
            throw new IOException("SHA-256 unavailable", e);
        }
        try {
            try (OutputStream out = new FileOutputStream(temp)) {
                copy(in, out, digest);
            }
            checkRom(temp);
            String hash = hex(digest.digest());
            File existing = findByHash(hash);
            if (existing != null) {
                temp.delete();
                return existing;
            }
            if (!temp.renameTo(target)) {
                throw new IOException("failed to replace " + target);
            }
            writeHash(target, hash);
            return target;
        } catch (IOException e) {
            temp.delete();
            throw e;
        }
    }

    private File findByHash(String hash) {
        File dir = romDirOrNull();
        if (dir == null) {
            return null;
        }
        File[] files = dir.listFiles();
        if (files == null) {
            return null;
        }
        for (File file : files) {
            if (isRom(file) && hash.equals(hashFor(file))) {
                return file;
            }
        }
        return null;
    }

    private static File hashSidecar(File rom) {
        return new File(rom.getParentFile(), rom.getName() + ".sha256");
    }

    private static String hashFor(File rom) {
        File sidecar = hashSidecar(rom);
        if (sidecar.isFile()) {
            try (InputStream in = new FileInputStream(sidecar)) {
                byte[] data = new byte[64];
                int read = in.read(data);
                if (read == 64) {
                    String text = new String(data, StandardCharsets.US_ASCII);
                    if (isHash(text)) {
                        return text;
                    }
                }
            } catch (IOException e) {
                Log.w(TAG, "failed to read hash for " + rom, e);
            }
        }
        String hash = hashFile(rom);
        if (hash != null) {
            writeHash(rom, hash);
        }
        return hash;
    }

    private static boolean isHash(String text) {
        if (text.length() != 64) {
            return false;
        }
        for (int i = 0; i < text.length(); ++i) {
            char c = text.charAt(i);
            if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'))) {
                return false;
            }
        }
        return true;
    }

    private static String hashFile(File file) {
        try (InputStream in = new FileInputStream(file)) {
            MessageDigest digest = MessageDigest.getInstance("SHA-256");
            byte[] buffer = new byte[64 * 1024];
            int read;
            while ((read = in.read(buffer)) != -1) {
                digest.update(buffer, 0, read);
            }
            return hex(digest.digest());
        } catch (IOException | NoSuchAlgorithmException e) {
            Log.w(TAG, "failed to hash " + file, e);
            return null;
        }
    }

    private static void writeHash(File rom, String hash) {
        try (OutputStream out = new FileOutputStream(hashSidecar(rom))) {
            out.write(hash.getBytes(StandardCharsets.US_ASCII));
        } catch (IOException e) {
            Log.w(TAG, "failed to cache hash for " + rom, e);
        }
    }

    private static String hex(byte[] bytes) {
        final char[] digits = "0123456789abcdef".toCharArray();
        char[] text = new char[bytes.length * 2];
        for (int i = 0; i < bytes.length; ++i) {
            int value = bytes[i] & 0xFF;
            text[i * 2] = digits[value >> 4];
            text[i * 2 + 1] = digits[value & 0x0F];
        }
        return new String(text);
    }

    private static File savePath(File rom, String extension) {
        String hash = hashFor(rom);
        return hash != null ? new File(rom.getParentFile(), hash + extension) : null;
    }

    private static String saveName(File rom) {
        String name = rom.getName();
        int dot = name.lastIndexOf('.');
        return dot > 0 ? name.substring(0, dot) : name;
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
        checkContentUri(uri);
        InputStream in = getContentResolver().openInputStream(uri);
        if (in == null) {
            throw new IOException("failed to open " + uri);
        }
        return in;
    }

    private static void checkContentUri(Uri uri) throws IOException {
        if (uri == null || !ContentResolver.SCHEME_CONTENT.equals(uri.getScheme())) {
            throw new IOException("unsupported URI: " + uri);
        }
        String path = uri.getPath();
        if (path == null) {
            return;
        }
        String normalized = Build.VERSION.SDK_INT >= Build.VERSION_CODES.O
            ? normalizePathApi26(path)
            : normalizePath(path);
        if (normalized.startsWith("/data")) {
            throw new IOException("refusing private URI: " + uri);
        }
    }

    @TargetApi(Build.VERSION_CODES.O)
    private static String normalizePathApi26(String path) {
        return java.nio.file.Paths.get(path).normalize().toString();
    }

    private static String normalizePath(String path) {
        String[] parts = path.split("/");
        StringBuilder out = new StringBuilder();
        int skip = 0;
        for (int i = parts.length - 1; i >= 0; --i) {
            String part = parts[i];
            if (part.isEmpty() || part.equals(".")) {
                continue;
            }
            if (part.equals("..")) {
                ++skip;
                continue;
            }
            if (skip > 0) {
                --skip;
                continue;
            }
            out.insert(0, "/" + part);
        }
        return out.length() == 0 ? "/" : out.toString();
    }

    private static void copy(InputStream in, OutputStream out) throws IOException {
        copy(in, out, null);
    }

    private static void copy(InputStream in, OutputStream out, MessageDigest digest)
            throws IOException {
        byte[] buffer = new byte[64 * 1024];
        int read;
        while ((read = in.read(buffer)) != -1) {
            out.write(buffer, 0, read);
            if (digest != null) {
                digest.update(buffer, 0, read);
            }
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

    private String displayName(Uri uri) throws IOException {
        checkContentUri(uri);
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

    private static String safeName(String name) throws IOException {
        if (name == null || name.isEmpty() || name.equals(".") || name.equals("..")
                || name.contains("..") || name.contains("/") || name.contains("\\")) {
            throw new IOException("invalid file name: " + name);
        }
        return name;
    }

    static String storedRom(Context context) {
        return context.getSharedPreferences(PREFS, MODE_PRIVATE)
                .getString(PREF_ROM_PATH, null);
    }

    private void launchGame(String path) {
        if (MainActivity.isRunning()) {
            // SDL has process-wide state; don't start a second emulator.
            Toast.makeText(this, R.string.game_already_running, Toast.LENGTH_SHORT).show();
            return;
        }
        remember(new File(path));
        Intent intent = new Intent(this, MainActivity.class);
        intent.putExtra(EXTRA_ROM_PATH, path);
        startActivity(intent);
        // Keep the library below the game so Back returns to the list.
    }
}
