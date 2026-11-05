package space.deci.baddmg;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class RomNameTest {
    @Test
    public void acceptsGameBoyExtensions() {
        assertTrue(RomPickerActivity.isRomName("game.gb"));
        assertTrue(RomPickerActivity.isRomName("game.gbc"));
        assertTrue(RomPickerActivity.isRomName("GAME.GB"));
    }

    @Test
    public void rejectsEverythingElse() {
        assertFalse(RomPickerActivity.isRomName("3x4_.jpg"));
        assertFalse(RomPickerActivity.isRomName("game.sav"));
        assertFalse(RomPickerActivity.isRomName("game.zip"));
        assertFalse(RomPickerActivity.isRomName("game.gb.tmp"));
        assertFalse(RomPickerActivity.isRomName("game.gb.sha256"));
        assertFalse(RomPickerActivity.isRomName("game"));
    }
}
