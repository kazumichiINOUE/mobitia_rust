use anyhow::Result;
use eframe::egui;

// Define application modules
mod app;
mod cli;
mod lidar;

use app::MyApp;

fn main() -> Result<(), eframe::Error> {
    let native_options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_maximized(true)
            .with_min_inner_size([400.0, 300.0]),
        ..Default::default()
    };
    eframe::run_native(
        "Mobitia 2-Pane Prototype",
        native_options,
        Box::new(|cc| {
            // --- Font setup ---
            let mut fonts = egui::FontDefinitions::default();

            // Install Source Han Sans font
            fonts.font_data.insert(
                "source_han_sans".to_owned(),
                egui::FontData::from_static(include_bytes!(
                    "../assets/fonts/SourceHanSans-Regular.otf"
                )),
            );

            // Install Hack Nerd Font
            fonts.font_data.insert(
                "hack_nerd".to_owned(),
                egui::FontData::from_static(include_bytes!(
                    "../assets/fonts/HackNerdFont-Regular.ttf"
                )),
            );

            // Set Hack Nerd Font as the first priority for both proportional and monospace fonts
            fonts
                .families
                .entry(egui::FontFamily::Proportional)
                .or_default()
                .insert(0, "hack_nerd".to_owned());
            fonts
                .families
                .entry(egui::FontFamily::Monospace)
                .or_default()
                .insert(0, "hack_nerd".to_owned());

            // Set Source Han Sans as a fallback font
            fonts
                .families
                .entry(egui::FontFamily::Proportional)
                .or_default()
                .push("source_han_sans".to_owned());
            fonts
                .families
                .entry(egui::FontFamily::Monospace)
                .or_default()
                .push("source_han_sans".to_owned());

            cc.egui_ctx.set_fonts(fonts);
            // --- End of font setup ---
            cc.egui_ctx.set_visuals(egui::Visuals::dark());
            let mut style = (*cc.egui_ctx.style()).clone();
            style.text_styles.insert(
                egui::TextStyle::Monospace,
                egui::FontId::proportional(18.0), // Adjust font size here
            );
            cc.egui_ctx.set_style(style);

            Box::new(MyApp::new(cc))
        }),
    )
}
