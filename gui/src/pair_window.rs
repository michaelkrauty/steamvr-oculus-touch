// SPDX-License-Identifier: Apache-2.0

use crate::pair_client::{Client, Controller, Event, Request};
use crate::{
    model::Level,
    window::{card, color, dot, title, weak},
};
use eframe::egui;
use std::collections::BTreeMap;

#[derive(Default)]
struct State {
    status: String,
    errors: Vec<String>,
    before: Option<(String, Vec<Controller>)>,
    after: Option<(String, Vec<Controller>)>,
    offers: BTreeMap<Controller, Offer>,
    listening: bool,
    finishing: bool,
    recovery: bool,
    done: bool,
    pending: bool,
}

struct Offer {
    firmware: String,
    paired: bool,
    also_paired: bool,
}

impl State {
    fn receive(&mut self, event: Event) {
        match event {
            Event::Status(status) => {
                self.listening = false;
                self.status = status;
            }
            Event::Snapshot {
                after,
                headset,
                controllers,
            } => {
                let snapshot = if after {
                    &mut self.after
                } else {
                    &mut self.before
                };
                *snapshot = Some((headset, controllers));
            }
            Event::Listening => {
                self.listening = !self.finishing;
                if !self.finishing {
                    self.status = "Listening for Touch controllers…".into();
                }
            }
            Event::Broadcast(controller, firmware, paired, also_paired) => {
                self.offers.insert(
                    controller,
                    Offer {
                        firmware,
                        paired,
                        also_paired,
                    },
                );
            }
            Event::Paired(controller, result) => {
                self.pending = false;
                match result {
                    Ok(()) => {
                        for (key, offer) in &mut self.offers {
                            if key.hand == controller.hand {
                                if key == &controller {
                                    offer.paired = true;
                                } else {
                                    offer.also_paired = true;
                                }
                            }
                        }
                    }
                    Err(error) => self.errors.push(error),
                }
            }
            Event::Error(error, recovery) => {
                self.errors.push(error);
                self.recovery |= recovery;
            }
            Event::Done => self.done = true,
            Event::Finished => {
                self.listening = false;
                self.status = if self.done {
                    "Finished. The headset is in normal mode."
                } else {
                    "Pairing did not complete."
                }
                .into();
            }
        }
    }
}

struct App {
    client: Option<Client>,
    state: State,
}

fn snapshot(ui: &mut egui::Ui, heading: &str, data: &(String, Vec<Controller>)) {
    title(ui, heading);
    ui.label(weak(format!("Headset radio: {}", data.0)));
    for (hand, label) in [("left", "Left"), ("right", "Right")] {
        let address = data
            .1
            .iter()
            .find(|c| c.hand == hand)
            .map_or("No reply (asleep, out of range, or not paired)", |c| {
                c.address.as_str()
            });
        ui.label(format!("{label}: {address}"));
    }
}

impl eframe::App for App {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        let ctx = ui.ctx().clone();
        if let Some(client) = &self.client {
            let mut finished = false;
            for event in client.events.try_iter() {
                finished |= matches!(event, Event::Finished);
                self.state.receive(event);
            }
            if finished {
                self.client = None;
            }
        }
        let state = &mut self.state;
        egui::CentralPanel::default().show(ui, |ui| {
            egui::ScrollArea::vertical().show(ui, |ui| {
                card(ui, |ui| {
                title(ui, "Pair Touch controllers");
                ui.label("Left controller: Hold Menu (☰) and Y until the light blinks.");
                ui.label("Right controller: Hold Oculus and B until the light blinks.");
                ui.label(weak("Keep the other controllers asleep or remove their batteries while pairing."));
                });
                card(ui, |ui| {
                    let level = if !state.errors.is_empty() { Level::Bad }
                        else if state.done || state.listening { Level::Good }
                        else if self.client.is_some() { Level::Warning }
                        else { Level::Quiet };
                    ui.horizontal(|ui| {
                        dot(ui, level);
                        ui.colored_label(color(ui, level), &state.status);
                    });
                    for error in &state.errors { ui.colored_label(color(ui, Level::Bad), error); }
                });
                if let Some(before) = &state.before {
                    card(ui, |ui| snapshot(ui, "Paired before", before));
                }
                if !state.offers.is_empty() {
                    card(ui, |ui| {
                    title(ui, "Controllers asking to pair");
                    for (controller, offer) in &state.offers {
                        ui.horizontal_wrapped(|ui| {
                            let label = if controller.hand == "left" { "Left" } else { "Right" };
                            ui.label(format!("{label}: {}", controller.address));
                            ui.label(weak(format!("Firmware {}", offer.firmware)));
                            if offer.paired {
                                ui.strong("Already paired");
                            } else if let Some(client) = &self.client {
                                if ui.add_enabled(state.listening && !state.pending, egui::Button::new("Pair")).clicked() {
                                    state.pending = true;
                                    client.request(Request::Pair(controller.clone()));
                                }
                            }
                        });
                        if offer.also_paired && !offer.paired {
                            ui.label(weak(concat!(
                                "A controller already answered for this hand. Pairing does not unpair it. ",
                                "Keep the unused controller asleep or remove its battery."
                            )));
                        }
                    }
                    });
                }
                if let Some(after) = &state.after {
                    card(ui, |ui| snapshot(ui, "Paired after", after));
                }
                ui.separator();
                ui.horizontal(|ui| {
                    if let Some(client) = &self.client {
                        if !state.finishing {
                            for (label, request) in [("Done", Request::Done), ("Cancel", Request::Cancel)] {
                                if ui.add_enabled(label == "Cancel" || (state.listening && !state.pending), egui::Button::new(label)).clicked() {
                                    client.request(request);
                                    state.listening = false;
                                    state.finishing = true;
                                    state.status = "Ending the pairing session…".into();
                                }
                            }
                        }
                    } else {
                        if state.recovery && ui.button("Return headset to normal mode").clicked() {
                            state.recovery = false;
                            state.done = false;
                            state.errors.clear();
                            state.finishing = true;
                            self.client = Some(Client::start(true, ctx.clone()));
                        }
                        if ui.button("Close").clicked() {
                            ctx.send_viewport_cmd(egui::ViewportCommand::Close);
                        }
                    }
                });
            });
        });
    }
}

pub fn run() {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_app_id(crate::window::APP_ID)
            .with_inner_size([620.0, 525.0])
            .with_icon(crate::window::icon_data(crate::icon::Tint::Ready)),
        ..Default::default()
    };
    if let Err(error) = eframe::run_native(
        "Pair Touch controllers",
        options,
        Box::new(|cc| {
            crate::window::configure_style(&cc.egui_ctx);
            crate::tray::listen_for_activation(&cc.egui_ctx);
            Ok(Box::new(App {
                client: Some(Client::start(false, cc.egui_ctx.clone())),
                state: State::default(),
            }))
        }),
    ) {
        eprintln!("cv1touch-gui: {error}");
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pairing_updates_badges_and_same_hand_warning() {
        let mut state = State::default();
        let first = Controller {
            hand: "left",
            address: "12345678".into(),
        };
        let second = Controller {
            hand: "left",
            address: "12345679".into(),
        };
        state.receive(Event::Broadcast(first.clone(), "1".into(), false, false));
        state.receive(Event::Broadcast(second.clone(), "1".into(), false, false));
        state.receive(Event::Paired(first.clone(), Ok(())));
        assert!(state.offers[&first].paired);
        assert!(state.offers[&second].also_paired);
        state.receive(Event::Error("Could not enter pairing mode".into(), true));
        state.receive(Event::Finished);
        assert!(state.recovery);
        assert!(!state.listening);
    }
}
