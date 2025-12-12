use eframe::egui;

pub struct TableDemo;

impl TableDemo {
    pub fn new() -> Self {
        Self
    }

    pub fn update_and_draw(&mut self, ui: &mut egui::Ui) {
        // --- Widgetベースのデモ ---
        ui.centered_and_justified(|ui| {
            egui::Grid::new("demo_table_grid")
                .num_columns(2)
                .spacing([40.0, 4.0])
                .striped(true)
                .show(ui, |ui| {
                    ui.label("Parameter 1:");
                    ui.label("Value A");
                    ui.end_row();

                    ui.label("Parameter 2:");
                    ui.label("Value B");
                    ui.end_row();

                    ui.label("A much longer parameter name:");
                    ui.label("Some other value C");
                    ui.end_row();

                    ui.label("Status:");
                    ui.label(egui::RichText::new("OK").color(egui::Color32::GREEN));
                    ui.end_row();
                });
        });
    }
}