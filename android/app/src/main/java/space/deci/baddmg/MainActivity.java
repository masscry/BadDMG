package space.deci.baddmg;

import android.os.Bundle;

import org.libsdl.app.SDLActivity;

import java.lang.ref.WeakReference;

/**
 * Runs the emulator. RomPickerActivity imports the ROM and starts this
 * activity with its path.
 */
public class MainActivity extends SDLActivity {
    // SDL keeps global state, so only one emulator instance can exist at a
    // time. The library checks this before starting another game.
    private static WeakReference<MainActivity> sCurrent;

    static boolean isRunning() {
        MainActivity current = sCurrent != null ? sCurrent.get() : null;
        // Keep the slot busy until the old SDL activity is fully destroyed.
        return current != null && !current.isDestroyed();
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        sCurrent = new WeakReference<>(this);
        super.onCreate(savedInstanceState);
    }

    @Override
    protected void onDestroy() {
        MainActivity current = sCurrent != null ? sCurrent.get() : null;
        if (current == this) {
            sCurrent = null;
        }
        super.onDestroy();
    }

    @Override
    protected String[] getArguments() {
        String rom = getIntent().getStringExtra(RomPickerActivity.EXTRA_ROM_PATH);
        if (rom == null) {
            rom = RomPickerActivity.storedRom(this);
        }
        return rom != null ? new String[] {"-c", rom} : new String[0];
    }
}
