package space.deci.baddmg;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertThrows;

import org.junit.Test;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;

/** The import cap is the only thing bounding untrusted file hand-offs. */
public class RomImportLimitTest {
    @Test
    public void acceptsFileAtLimit() throws IOException {
        ByteArrayOutputStream out = new ByteArrayOutputStream();
        RomPickerActivity.copy(
                new ByteArrayInputStream(new byte[(int) RomPickerActivity.MAX_ROM_SIZE]),
                out, null, RomPickerActivity.MAX_ROM_SIZE);
        assertEquals(RomPickerActivity.MAX_ROM_SIZE, out.size());
    }

    @Test
    public void rejectsFileOverLimit() {
        byte[] data = new byte[(int) RomPickerActivity.MAX_ROM_SIZE + 1];
        assertThrows(IOException.class, () -> RomPickerActivity.copy(
                new ByteArrayInputStream(data), new ByteArrayOutputStream(), null,
                RomPickerActivity.MAX_ROM_SIZE));
    }
}
