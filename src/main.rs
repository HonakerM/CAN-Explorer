#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

mod app;
mod bus;
mod candump;
mod device;
mod msg;
mod symbols;

fn main() -> eframe::Result<()> {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title("CAN Explorer")
            .with_inner_size([1400.0, 850.0])
            .with_min_inner_size([800.0, 500.0]),
        ..Default::default()
    };
    eframe::run_native(
        "CAN Explorer",
        options,
        Box::new(|cc| Ok(Box::new(app::App::new(cc)))),
    )
}
