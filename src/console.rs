use {
    crate::display::FramebufDisplay,
    core::fmt::Write,
    embedded_graphics::{
        mono_font::{ascii::FONT_9X18, MonoTextStyle},
        pixelcolor::Rgb888,
        prelude::*,
        text::Text,
    },
};

pub struct DisplayAndUartConsole<U: Write> {
    uart: U,
    display: FramebufDisplay,
    curr_pos_y: u32,
    curr_pos_x: u32,
}

impl<U: Write> DisplayAndUartConsole<U> {
    pub fn new(mut display: FramebufDisplay, uart: U) -> DisplayAndUartConsole<U> {
        display.clear(Rgb888::BLACK).expect("can't clear display");

        Self {
            uart,
            display,
            curr_pos_y: 0,
            curr_pos_x: 0,
        }
    }

    fn add_line(&mut self, line: &str) {
        self.uart.write_str(line).ok();

        let font = &FONT_9X18;
        let line_height = font.character_size.height;
        let char_width = font.character_size.width;
        let line_width = char_width * line.len() as u32;

        if self.curr_pos_y > self.display.height() as u32 {
            self.display.scroll_up(line_height as usize);
            self.curr_pos_y -= line_height;
        }

        // Text wrapping
        if self.curr_pos_x + line_width >= self.display.width() as u32 {
            let mut curr_line_width = line_width;
            let mut line_pos = 0usize;
            while self.curr_pos_x + curr_line_width >= self.display.width() as u32 {
                let free_space_px = self.display.width() - self.curr_pos_x as usize;
                let free_space_chars = free_space_px / char_width as usize;

                self.draw_line(unsafe { line.get_unchecked(line_pos..free_space_chars) });
                curr_line_width -= free_space_px as u32;
                line_pos += free_space_chars;
                self.curr_pos_y += line_height;

                if self.curr_pos_y > self.display.height() as u32 {
                    self.display.scroll_up(line_height as usize);
                    self.curr_pos_y -= line_height;
                }
                self.curr_pos_x = 0;
            }

            self.draw_line(unsafe { line.get_unchecked(line_pos..line.len()) });
        } else {
            self.draw_line(line);
        }

        if line.contains('\n') {
            self.curr_pos_y += line_height;
            self.curr_pos_x = 0;
        } else {
            self.curr_pos_x += line_width;
        }
    }

    fn draw_line(&mut self, line: &str) {
        let pos = Point::new(self.curr_pos_x as i32, self.curr_pos_y as i32);
        Text::new(line, pos, MonoTextStyle::new(&FONT_9X18, Rgb888::WHITE))
            .draw(&mut self.display)
            .expect("can't draw line");
    }
}

impl<U: Write> Write for DisplayAndUartConsole<U> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.add_line(s);
        Ok(())
    }
}
