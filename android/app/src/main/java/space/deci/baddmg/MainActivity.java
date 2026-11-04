package space.deci.baddmg;

import org.libsdl.app.SDLActivity;

/**
 * Runs the emulator. RomPickerActivity imports the ROM and starts this
 * activity with its path.
 */
public class MainActivity extends SDLActivity {
    @Override
    protected String[] getArguments() {
        String rom = getIntent().getStringExtra(RomPickerActivity.EXTRA_ROM_PATH);
        if (rom == null) {
            rom = RomPickerActivity.storedRom(this);
        }
        return rom != null ? new String[] {"-c", rom} : new String[0];
    }
}
