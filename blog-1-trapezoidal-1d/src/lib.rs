use std::cell::Cell;
use std::rc::Rc;
use trajectory_planner::TrajectorySegment;
use wasm_bindgen::prelude::*;
use wasm_bindgen::JsCast;

#[wasm_bindgen(start)]
pub fn start() -> Result<(), JsValue> {
    let document = web_sys::window().unwrap().document().unwrap();
    let canvas = document
        .create_element("canvas")?
        .dyn_into::<web_sys::HtmlCanvasElement>()?;
    document.body().unwrap().append_child(&canvas)?;

    let width = 640;
    let height = 480;

    canvas.set_width(width);
    canvas.set_height(480);
    canvas.style().set_property("border", "solid")?;
    canvas.style().set_property("border-width", "1px")?;

    let context = canvas
        .get_context("2d")?
        .unwrap()
        .dyn_into::<web_sys::CanvasRenderingContext2d>()?;

    let num_steps = 100;
    let start = 0.0;
    let end = 10.0;
    let padding_left = 10.0;

    let segment = TrajectorySegment::new(start, end, 5.0, 2.0);

    web_sys::console::log_1(&format!("Traj: {:#?}", segment).into());

    // Position
    context.begin_path();
    context.move_to(padding_left, (height / 2) as f64);
    context.set_stroke_style(&("#000".into()));

    for i in 0..num_steps {
        let time = segment.duration * i as f32 / num_steps as f32;

        let y = (height / 2) as f32 - segment.position(time) * 10.0;

        web_sys::console::log_1(&format!("Pos T: {}, Y: {}", time, y).into());

        context.line_to(
            padding_left + (width / num_steps) as f64 * i as f64,
            y as f64,
        );
    }

    context.stroke();
    context.close_path();

    // Velocity
    context.begin_path();
    context.move_to(padding_left, (height / 2) as f64);
    context.set_stroke_style(&("#f00".into()));

    for i in 0..num_steps {
        let time = segment.duration * i as f32 / num_steps as f32;

        let y = (height / 2) as f32 - segment.velocity(time) * 10.0;

        web_sys::console::log_1(&format!("Vel T: {}, Y: {}", time, y).into());

        context.line_to(
            padding_left + (width / num_steps) as f64 * i as f64,
            y as f64,
        );
    }

    context.stroke();
    context.close_path();

    // Acceleration
    context.begin_path();
    context.move_to(padding_left, (height / 2) as f64);
    context.set_stroke_style(&("#00f".into()));

    for i in 0..num_steps {
        let time = segment.duration * i as f32 / num_steps as f32;

        let y = (height / 2) as f32 - segment.acceleration(time) * 10.0;

        web_sys::console::log_1(&format!("Vel T: {}, Y: {}", time, y).into());

        context.line_to(
            padding_left + (width / num_steps) as f64 * i as f64,
            y as f64,
        );
    }

    context.stroke();
    context.close_path();

    // let context = Rc::new(context);
    // let pressed = Rc::new(Cell::new(false));

    // {
    //     let context = context.clone();
    //     let pressed = pressed.clone();
    //     let closure = Closure::wrap(Box::new(move |event: web_sys::MouseEvent| {
    //         context.begin_path();
    //         context.move_to(event.offset_x() as f64, event.offset_y() as f64);
    //         pressed.set(true);
    //     }) as Box<dyn FnMut(_)>);
    //     canvas.add_event_listener_with_callback("mousedown", closure.as_ref().unchecked_ref())?;
    //     closure.forget();
    // }
    // {
    //     let context = context.clone();
    //     let pressed = pressed.clone();
    //     let closure = Closure::wrap(Box::new(move |event: web_sys::MouseEvent| {
    //         if pressed.get() {
    //             context.line_to(event.offset_x() as f64, event.offset_y() as f64);
    //             context.stroke();
    //             context.begin_path();
    //             context.move_to(event.offset_x() as f64, event.offset_y() as f64);
    //         }
    //     }) as Box<dyn FnMut(_)>);
    //     canvas.add_event_listener_with_callback("mousemove", closure.as_ref().unchecked_ref())?;
    //     closure.forget();
    // }
    // {
    //     let context = context.clone();
    //     let pressed = pressed.clone();
    //     let closure = Closure::wrap(Box::new(move |event: web_sys::MouseEvent| {
    //         pressed.set(false);
    //         context.line_to(event.offset_x() as f64, event.offset_y() as f64);
    //         context.stroke();
    //     }) as Box<dyn FnMut(_)>);
    //     canvas.add_event_listener_with_callback("mouseup", closure.as_ref().unchecked_ref())?;
    //     closure.forget();
    // }

    Ok(())
}
