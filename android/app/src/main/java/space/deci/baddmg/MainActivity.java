package space.deci.baddmg;

import android.os.Build;
import android.os.Bundle;
import android.os.VibrationAttributes;
import android.os.VibrationEffect;
import android.os.Vibrator;

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

    // Rumble cartridges. SDL's haptic API vibrates with the generic touch
    // usage, so the system scales it with the "touch feedback" setting.
    // Android 14+ has a usage meant for game rumble; use it when possible.
    // Both methods are called from native code (rumble.cc).
    private Vibrator mRumbleVibrator;
    private boolean mRumbleUnavailable = Build.VERSION.SDK_INT < Build.VERSION_CODES.UPSIDE_DOWN_CAKE;

    /**
     * Vibrates for durationMs with the physical emulation usage. Returns false
     * when it cannot (old Android, no vibrator): the native side then falls
     * back to SDL's haptic API.
     */
    public boolean rumblePulse(int durationMs) {
        if (mRumbleUnavailable) {
            return false;
        }
        try {
            if (mRumbleVibrator == null) {
                mRumbleVibrator = getSystemService(Vibrator.class);
            }
            if (mRumbleVibrator == null || !mRumbleVibrator.hasVibrator()) {
                mRumbleUnavailable = true;
                return false;
            }
            mRumbleVibrator.vibrate(
                    VibrationEffect.createOneShot(durationMs, 255),
                    VibrationAttributes.createForUsage(VibrationAttributes.USAGE_PHYSICAL_EMULATION));
            return true;
        } catch (RuntimeException | LinkageError e) {
            mRumbleUnavailable = true;
            return false;
        }
    }

    public void rumbleStop() {
        if (mRumbleVibrator != null) {
            mRumbleVibrator.cancel();
        }
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
