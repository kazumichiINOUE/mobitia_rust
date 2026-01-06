use eframe::egui;
use nalgebra::{Isometry2, Point2};

use crate::config::Config;

#[derive(Default)]
pub struct MapScreen {}

impl MapScreen {
    pub fn new() -> Self {
        Self {}
    }

        #[allow(clippy::too_many_arguments)]

        pub fn draw(

            &mut self,

            ui: &mut egui::Ui,

            config: &Config,

            lidar_draw_rect: &mut Option<egui::Rect>,

            current_robot_pose: &Isometry2<f32>,

            slam_map_bounding_box: &Option<egui::Rect>,

            robot_trajectory: &[(egui::Pos2, f32)],

            current_map_points: &[(Point2<f32>, f64)],

            map_loading_complete: bool,

        ) {

            ui.heading("Map Mode");

    

            let (response, painter) = ui.allocate_painter(ui.available_size(), egui::Sense::hover());

            let rect = response.rect;

            *lidar_draw_rect = Some(rect);

            painter.rect_filled(rect, 0.0, egui::Color32::from_rgb(20, 20, 20));

    

            // Draw robot pose

            let robot_pose = current_robot_pose;

            // 描画範囲を決定

            let map_view_rect = if let Some(bounds) = slam_map_bounding_box {

                // ロードした地図がある場合、そのバウンディングボックスを表示範囲とする

                let new_bounds = bounds.expand(bounds.width() * 0.1); // 先に少しマージンを追加

    

                let screen_aspect = rect.width() / rect.height();

                if !screen_aspect.is_nan() {

                    let bounds_aspect = new_bounds.width() / new_bounds.height();

                    let center = new_bounds.center();

                    let (width, height) = if bounds_aspect > screen_aspect {

                        // バウンディングボックスの方が横長 -> 高さを増やす

                        (new_bounds.width(), new_bounds.width() / screen_aspect)

                    } else {

                        // バウンディングボックスの方が縦長 -> 幅を増やす

                        (new_bounds.height() * screen_aspect, new_bounds.height())

                    };

                    egui::Rect::from_center_size(center, egui::vec2(width, height))

                } else {

                    new_bounds

                }

            } else {

                // 地図がまだない場合

                let robot_center_world = egui::pos2(robot_pose.translation.x, robot_pose.translation.y);

                let map_view_size = config.slam.online_slam_view_size; // 追従時の表示サイズ

                egui::Rect::from_center_size(

                    robot_center_world,

                    egui::vec2(map_view_size, map_view_size),

                )

            };

    

            // Y軸を反転させるため、map_view_rectのYのmin/maxを入れ替える

            let mut inverted_map_view_rect = map_view_rect;

            inverted_map_view_rect.min.y = map_view_rect.max.y;

            inverted_map_view_rect.max.y = map_view_rect.min.y;

            let to_screen = egui::emath::RectTransform::from_to(inverted_map_view_rect, rect);

    

            // Draw the map points

            // 各セルの画面上でのサイズを固定値として定義 (以前の円の半径2.0に合わせる)

            const CELL_FIXED_SCREEN_SIZE: f32 = 2.0;

            let cell_fixed_size_vec = egui::vec2(CELL_FIXED_SCREEN_SIZE, CELL_FIXED_SCREEN_SIZE);

    

            for (point, probability) in current_map_points {

                // Right = +X, Up = +Y

                let screen_pos = to_screen.transform_pos(egui::pos2(point.x, point.y));

                if rect.contains(screen_pos) {

                    let prob_f32 = *probability as f32;

    

                    // Probability == 0.5 is the initial state (unknown), so we don't draw it.

                    if (prob_f32 - 0.5).abs() < 1e-6 {

                        continue;

                    }

    

                    // 各セルの画面上での矩形を計算

                    let cell_rect = egui::Rect::from_center_size(screen_pos, cell_fixed_size_vec);

    

                    if prob_f32 > 0.5 {

                        let intensity = (prob_f32 - 0.5) / 0.5; // Normalize 0.5-1.0 to 0-1

                        let color = egui::Color32::from_rgb(

                            (intensity * 100.0) as u8,

                            (intensity * 100.0) as u8,

                            (intensity * 255.0) as u8,

                        );

                        painter.rect_filled(cell_rect, 0.0, color);

                    } else {

                        let intensity = prob_f32 / 0.5;

                        const MAX_FREE_GRAY: u8 = 150;

                        const MIN_FREE_GRAY: u8 = 25;

                        let gray_value = MIN_FREE_GRAY

                            + ((MAX_FREE_GRAY - MIN_FREE_GRAY) as f32 * (1.0 - intensity)) as u8;

                        let color = egui::Color32::from_gray(gray_value);

                        painter.rect_filled(cell_rect, 0.0, color);

                    }

                }

            }

    

            // 軌跡の線と向き（三角形）を描画

            if robot_trajectory.len() > 1 {

                // 軌跡の線

                let trajectory_line_points: Vec<egui::Pos2> = robot_trajectory

                    .iter()

                    .map(|(world_pos, _angle)| to_screen.transform_pos(*world_pos))

                    .collect();

                let line_stroke = egui::Stroke::new(1.0, egui::Color32::WHITE);

                painter.add(egui::Shape::line(trajectory_line_points, line_stroke));

    

                // 各スキャン姿勢の中心位置を点で描画

                for (world_pos, _angle) in robot_trajectory.iter() {

                    let screen_pos = to_screen.transform_pos(*world_pos);

                    if rect.contains(screen_pos) {

                        painter.circle_filled(screen_pos, 2.0, egui::Color32::LIGHT_BLUE);

                    }

                }

    

                // 向きを示すくの字マーカー

                let chevron_color = egui::Color32::from_rgb(200, 200, 100); // 彩度を落とした黄色

                let chevron_stroke = egui::Stroke::new(1.0, chevron_color); // 細めの線

                let chevron_size = 8.0; // 小さめのサイズ

    

                for (i, (world_pos, angle)) in robot_trajectory.iter().enumerate() {

                    if i > 0 {

                        let center_screen = to_screen.transform_pos(*world_pos);

    

                        // くの字の頂点を定義

                        // 先端

                        let p1 = center_screen + egui::vec2(angle.cos(), -angle.sin()) * chevron_size;

                        // 左後ろの点 (先端から約135度)

                        let angle_left = *angle + (135.0f32).to_radians();

                        let p2 = center_screen

                            + egui::vec2(angle_left.cos(), -angle_left.sin()) * chevron_size * 0.7;

                        // 右後ろの点 (先端から約-135度)

                        let angle_right = *angle - (135.0f32).to_radians();

                        let p3 = center_screen

                            + egui::vec2(angle_right.cos(), -angle_right.sin()) * chevron_size * 0.7;

    

                        // 2本の線分でくの字を描画

                        painter.add(egui::Shape::line_segment([p2, p1], chevron_stroke));

                        painter.add(egui::Shape::line_segment([p3, p1], chevron_stroke));

                    }

                }

            }

    

                    // Draw robot pose (この部分は変更しない)

    

                    let robot_pos_on_screen = to_screen.transform_pos(egui::pos2(

    

                        robot_pose.translation.x,

    

                        robot_pose.translation.y,

    

                    ));

    

                    let angle = robot_pose.rotation.angle();

    

                    painter.circle_filled(robot_pos_on_screen, 5.0, egui::Color32::RED);

    

                    painter.line_segment(

    

                        [

    

                            robot_pos_on_screen,

    

                            robot_pos_on_screen + egui::vec2(angle.cos(), -angle.sin()) * 20.0,

    

                        ],

    

                        egui::Stroke::new(2.0, egui::Color32::RED),

    

                    );

    

            

    

                    // --- 地図読み込み完了後にバウンディングボックスと寸法を描画 ---

    

                    if map_loading_complete {

    

                        if let Some(bbox_world) = slam_map_bounding_box {

    

                            // ワールド座標の4隅を画面座標に変換

    

                            let p_top_left_world = egui::pos2(bbox_world.min.x, bbox_world.max.y);

    

                            let p_top_right_world = egui::pos2(bbox_world.max.x, bbox_world.max.y);

    

                            let p_bottom_left_world = egui::pos2(bbox_world.min.x, bbox_world.min.y);

    

                            let p_bottom_right_world = egui::pos2(bbox_world.max.x, bbox_world.min.y);

    

            

    

                            let p_top_left_screen = to_screen.transform_pos(p_top_left_world);

    

                            let p_top_right_screen = to_screen.transform_pos(p_top_right_world);

    

                            let p_bottom_left_screen = to_screen.transform_pos(p_bottom_left_world);

    

                            let p_bottom_right_screen = to_screen.transform_pos(p_bottom_right_world);

    

            

    

                            // 画面座標でのバウンディングボックスを構築 (egui::Rectはmin <= maxを期待)

    

                            let bbox_screen = egui::Rect::from_min_max(

    

                                egui::pos2(p_top_left_screen.x.min(p_bottom_right_screen.x), p_top_left_screen.y.min(p_bottom_right_screen.y)),

    

                                egui::pos2(p_top_left_screen.x.max(p_bottom_right_screen.x), p_top_left_screen.y.max(p_bottom_right_screen.y)),

    

                            );

    

            

    

                            // 枠線は描画しない (ユーザーの要望)

    

            

    

                            let text_color = egui::Color32::WHITE;

    

                            let bg_color = egui::Color32::from_black_alpha(150);

    

                            let line_color = egui::Color32::WHITE;

    

                            let font_id = egui::FontId::monospace(12.0);

    

                            let text_offset_from_line = -5.0; // テキストと寸法線の間のオフセット

    

                            let extension_line_length = 10.0; // 引き出し線の長さ

    

                            let dimension_line_offset = 25.0; // 寸法線がバウンディングボックスから離れる距離

    

                            let line_thickness = 1.0;

    

            

    

            

    

                            // --- 横幅の寸法 ---

    

                            let width_val = bbox_world.width();

    

                            let width_text = format!("{:.2}m", width_val);

    

            

    

                            // 寸法線のY座標 (バウンディングボックス下辺からdimension_line_offsetだけ離す)

    

                            let dimension_line_y_screen = bbox_screen.max.y + dimension_line_offset;

    

            

    

                            // 垂直引き出し線 (左)

    

                            painter.add(egui::Shape::line_segment([

    

                                egui::pos2(p_bottom_left_screen.x, p_bottom_left_screen.y),

    

                                egui::pos2(p_bottom_left_screen.x, dimension_line_y_screen + extension_line_length),

    

                            ], egui::Stroke::new(line_thickness, line_color)));

    

                            painter.add(egui::Shape::line_segment([

    

                                egui::pos2(p_bottom_left_screen.x, dimension_line_y_screen), // 寸法線

    

                                egui::pos2(p_bottom_left_screen.x, dimension_line_y_screen + extension_line_length),

    

                            ], egui::Stroke::new(line_thickness, line_color)));

    

            

    

            

    

                            // 垂直引き出し線 (右)

    

                            painter.add(egui::Shape::line_segment([

    

                                egui::pos2(p_bottom_right_screen.x, p_bottom_right_screen.y),

    

                                egui::pos2(p_bottom_right_screen.x, dimension_line_y_screen + extension_line_length),

    

                            ], egui::Stroke::new(line_thickness, line_color)));

    

                            painter.add(egui::Shape::line_segment([

    

                                egui::pos2(p_bottom_right_screen.x, dimension_line_y_screen), // 寸法線

    

                                egui::pos2(p_bottom_right_screen.x, dimension_line_y_screen + extension_line_length),

    

                            ], egui::Stroke::new(line_thickness, line_color)));

    

            

    

            

    

                            // 水平寸法線

    

                            painter.add(egui::Shape::line_segment([

    

                                egui::pos2(p_bottom_left_screen.x, dimension_line_y_screen),

    

                                egui::pos2(p_bottom_right_screen.x, dimension_line_y_screen),

    

                            ], egui::Stroke::new(line_thickness, line_color)));

    

            

    

                            // 横幅テキスト

    

                            let width_text_center_screen = egui::pos2(bbox_screen.center().x, dimension_line_y_screen - text_offset_from_line);

    

                            let width_text_galley = ui.fonts(|f| f.layout_no_wrap(width_text.clone(), font_id.clone(), text_color));

    

                            let width_text_rect = egui::Rect::from_center_size(width_text_center_screen, width_text_galley.size()).expand(2.0);

    

                                            painter.rect_filled(width_text_rect, 0.0, bg_color); // 角丸Rは0

    

                                            painter.galley(width_text_center_screen, width_text_galley);

    

            

    

                            // --- 縦幅の寸法 ---

    

                            let height_val = bbox_world.height();

    

                            let height_text = format!("{:.2}m", height_val);

    

            

    

                            // 寸法線のX座標 (バウンディングボックス左辺からdimension_line_offsetだけ離す)

    

                            let dimension_line_x_screen = bbox_screen.min.x - dimension_line_offset;

    

            

    

                            // 水平引き出し線 (下)

    

                            painter.add(egui::Shape::line_segment([

    

                                egui::pos2(p_bottom_left_screen.x, p_bottom_left_screen.y),

    

                                egui::pos2(dimension_line_x_screen - extension_line_length, p_bottom_left_screen.y),

    

                            ], egui::Stroke::new(line_thickness, line_color)));

    

                            painter.add(egui::Shape::line_segment([

    

                                egui::pos2(dimension_line_x_screen, p_bottom_left_screen.y), // 寸法線

    

                                egui::pos2(dimension_line_x_screen - extension_line_length, p_bottom_left_screen.y),

    

                            ], egui::Stroke::new(line_thickness, line_color)));

    

            

    

            

    

                            // 水平引き出し線 (上)

    

                            painter.add(egui::Shape::line_segment([

    

                                egui::pos2(p_top_left_screen.x, p_top_left_screen.y),

    

                                egui::pos2(dimension_line_x_screen - extension_line_length, p_top_left_screen.y),

    

                            ], egui::Stroke::new(line_thickness, line_color)));

    

                            painter.add(egui::Shape::line_segment([

    

                                egui::pos2(dimension_line_x_screen, p_top_left_screen.y), // 寸法線

    

                                egui::pos2(dimension_line_x_screen - extension_line_length, p_top_left_screen.y),

    

                            ], egui::Stroke::new(line_thickness, line_color)));

    

            

    

                            // 垂直寸法線

    

                            painter.add(egui::Shape::line_segment([

    

                                egui::pos2(dimension_line_x_screen, p_bottom_left_screen.y),

    

                                egui::pos2(dimension_line_x_screen, p_top_left_screen.y),

    

                            ], egui::Stroke::new(line_thickness, line_color)));

    

            

    

                            // 縦幅テキスト

    

                            let height_text_center_screen = egui::pos2(dimension_line_x_screen + text_offset_from_line, bbox_screen.center().y);

    

                            let height_text_galley = ui.fonts(|f| f.layout_no_wrap(height_text.clone(), font_id, text_color));

    

                            let height_text_rect = egui::Rect::from_center_size(height_text_center_screen, height_text_galley.size()).expand(2.0);

    

                                            painter.rect_filled(height_text_rect, 0.0, bg_color);

    

                                            painter.galley(height_text_center_screen, height_text_galley);

    

                        }

    

                    }

    

                }
}
