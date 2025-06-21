use bevy::prelude::*;
use bevy_ecs_tilemap::prelude::*;
use pathfinding::prelude::astar;
use rand::seq::SliceRandom;
use rand::thread_rng;
use std::time::Duration;

// --- 常量定义 (Constants) ---
const TILE_SIZE: f32 = 16.0;

const TILE_WALL: u32 = 0;
const TILE_PATH: u32 = 1;
const TILE_START: u32 = 2;
const TILE_EXIT: u32 = 3;

const Z_LAYER_MAZE: f32 = 0.0;
const Z_LAYER_PLAYER: f32 = 1.0;
const Z_LAYER_MONSTER: f32 = 1.0;

// --- 核心游戏状态机 (Core Game State Machine) ---
#[derive(Clone, Copy, Default, Eq, PartialEq, Debug, Hash, States)]
enum AppState {
    #[default]
    MainMenu,
    InGame,
}

// --- 组件定义 (Components) ---
#[derive(Component)]
struct Player;

#[derive(Component)]
struct Monster;

#[derive(Component, Deref, DerefMut, PartialEq, Eq, Clone, Copy, Hash, Debug)]
struct GridPosition(UVec2);

#[derive(Component)]
struct GameEntity; 

#[derive(Component)]
struct MenuEntity;

#[derive(Component)]
struct TimerText;

// --- 资源与数据结构 (Resources & Data Structures) ---
#[derive(Resource)]
struct MoveCooldown(Timer);

#[derive(Resource)]
struct MonsterAiTimer(Timer);

#[derive(Resource)]
struct GameTimer(Timer);

#[derive(Resource, Debug, Clone, Copy)]
pub struct GameConfig {
    pub difficulty: Difficulty,
}

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub enum Difficulty {
    Easy,
    #[default]
    Normal,
    Hard,
}

impl Difficulty {
    pub fn get_dimensions(&self) -> (u32, u32) {
        match self {
            Difficulty::Easy => (21, 15),
            Difficulty::Normal => (41, 31),
            Difficulty::Hard => (61, 51),
        }
    }
}

#[derive(Resource, Debug, Default)]
pub struct Maze {
    pub width: u32,
    pub height: u32,
    pub grid: Vec<Vec<Cell>>,
    pub start_pos: UVec2,
    pub exit_pos: UVec2,
    pub monster_start_pos: UVec2,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Cell {
    Wall,
    Path,
    Start,
    Exit,
}

// --- Bevy 应用入口 ---
fn main() {
    let mut move_cooldown = Timer::from_seconds(0.12, TimerMode::Once);
    move_cooldown.set_elapsed(Duration::from_secs_f32(1.0));

    App::new()
        .add_plugins(
            DefaultPlugins
                .set(WindowPlugin {
                    primary_window: Some(Window {
                        title: "Maze Challenge - Final Version!".into(),
                        resizable: false,
                        ..default()
                    }),
                    ..default()
                })
                .set(ImagePlugin::default_nearest()),
        )
        .add_plugins(TilemapPlugin)
        
        // --- 状态机设置 ---
        // 【关键修正】使用 v0.12.1 的正确方法 `add_state` 来初始化状态机
        .add_state::<AppState>()
        
        // --- 通用资源 ---
        .insert_resource(GameConfig { difficulty: Difficulty::default() })
        .insert_resource(MoveCooldown(move_cooldown))
        .insert_resource(MonsterAiTimer(Timer::from_seconds(0.5, TimerMode::Repeating)))
        .insert_resource(GameTimer(Timer::from_seconds(9999.0, TimerMode::Once)))
        
        // --- 主菜单状态系统 ---
        .add_systems(OnEnter(AppState::MainMenu), setup_main_menu)
        .add_systems(Update, main_menu_system.run_if(in_state(AppState::MainMenu)))
        .add_systems(OnExit(AppState::MainMenu), cleanup_menu)
        
        // --- 游戏进行中状态系统 ---
        .add_systems(OnEnter(AppState::InGame), spawn_level)
        .add_systems(Update, 
            (
                player_movement, 
                monster_ai_system, 
                update_timer_ui
            ).run_if(in_state(AppState::InGame))
        )
        .add_systems(OnExit(AppState::InGame), cleanup_game)
        
        .run();
}

// --- 主菜单系统 (Main Menu Systems) ---
fn setup_main_menu(mut commands: Commands, mut q_windows: Query<&mut Window>) {
    let mut window = q_windows.single_mut();
    window.resolution.set(500.0, 300.0);

    commands.spawn((Camera2dBundle::default(), MenuEntity));
    
    commands.spawn((
        MenuEntity,
        NodeBundle {
            style: Style {
                width: Val::Percent(100.0),
                height: Val::Percent(100.0),
                flex_direction: FlexDirection::Column,
                align_items: AlignItems::Center,
                justify_content: JustifyContent::Center,
                row_gap: Val::Px(15.0),
                ..default()
            },
            ..default()
        },
    )).with_children(|parent| {
        parent.spawn(TextBundle::from_section(
            "MAZE CHALLENGE",
            TextStyle { font_size: 48.0, color: Color::WHITE, ..default() },
        ));
        parent.spawn(TextBundle::from_section(
            "Select Difficulty:",
            TextStyle { font_size: 24.0, color: Color::GRAY, ..default() },
        ));
        parent.spawn(TextBundle::from_section(
            "[1] Easy   [2] Normal   [3] Hard",
            TextStyle { font_size: 20.0, color: Color::WHITE, ..default() },
        ));
    });
}

fn main_menu_system(
    mut game_config: ResMut<GameConfig>,
    mut next_state: ResMut<NextState<AppState>>,
    keyboard_input: Res<Input<KeyCode>>,
) {
    if keyboard_input.just_pressed(KeyCode::Key1) {
        game_config.difficulty = Difficulty::Easy;
        next_state.set(AppState::InGame);
    } else if keyboard_input.just_pressed(KeyCode::Key2) {
        game_config.difficulty = Difficulty::Normal;
        next_state.set(AppState::InGame);
    } else if keyboard_input.just_pressed(KeyCode::Key3) {
        game_config.difficulty = Difficulty::Hard;
        next_state.set(AppState::InGame);
    }
}

fn cleanup_menu(mut commands: Commands, query: Query<Entity, With<MenuEntity>>) {
    for entity in &query {
        commands.entity(entity).despawn_recursive();
    }
}

// --- 游戏核心系统 (In-Game Systems) ---

fn player_movement(
    mut next_state: ResMut<NextState<AppState>>,
    keyboard_input: Res<Input<KeyCode>>,
    maze: Res<Maze>,
    mut player_query: Query<(&mut Transform, &mut GridPosition), (With<Player>, Without<Monster>)>,
    monster_query: Query<&GridPosition, (With<Monster>, Without<Player>)>,
    time: Res<Time>,
    mut move_cooldown: ResMut<MoveCooldown>,
    mut game_timer: ResMut<GameTimer>,
) {
    move_cooldown.0.tick(time.delta());
    game_timer.0.tick(time.delta());

    if !move_cooldown.0.finished() {
        return;
    }

    if let Ok((mut player_transform, mut player_grid_pos)) = player_query.get_single_mut() {
        let mut target_grid_pos = **player_grid_pos;

        if keyboard_input.pressed(KeyCode::Up) || keyboard_input.pressed(KeyCode::W) {
            target_grid_pos.y += 1;
        } else if keyboard_input.pressed(KeyCode::Down) || keyboard_input.pressed(KeyCode::S) {
            target_grid_pos.y = target_grid_pos.y.saturating_sub(1);
        } else if keyboard_input.pressed(KeyCode::Left) || keyboard_input.pressed(KeyCode::A) {
            target_grid_pos.x = target_grid_pos.x.saturating_sub(1);
        } else if keyboard_input.pressed(KeyCode::Right) || keyboard_input.pressed(KeyCode::D) {
            target_grid_pos.x += 1;
        }

        if target_grid_pos != **player_grid_pos {
            if let Ok(monster_pos) = monster_query.get_single() {
                if GridPosition(target_grid_pos) == *monster_pos {
                    println!("\nGAME OVER! You ran into the monster. Returning to menu...");
                    game_timer.0.pause();
                    next_state.set(AppState::MainMenu);
                    return;
                }
            }
            
            if target_grid_pos.x < maze.width && target_grid_pos.y < maze.height {
                let target_cell = maze.grid[target_grid_pos.y as usize][target_grid_pos.x as usize];
                if target_cell != Cell::Wall {
                    *player_grid_pos = GridPosition(target_grid_pos);
                    let new_world_pos = grid_to_world_coordinates(*player_grid_pos, maze.width, maze.height);
                    player_transform.translation.x = new_world_pos.x;
                    player_transform.translation.y = new_world_pos.y;
                    move_cooldown.0.reset();

                    if target_cell == Cell::Exit {
                        println!("\nCongratulations! You Win in {:.2} seconds! Returning to menu...", game_timer.0.elapsed_secs());
                        game_timer.0.pause();
                        next_state.set(AppState::MainMenu);
                    }
                }
            }
        }
    }
}

fn monster_ai_system(
    mut next_state: ResMut<NextState<AppState>>,
    maze: Res<Maze>,
    player_query: Query<&GridPosition, (With<Player>, Without<Monster>)>,
    mut monster_query: Query<(&mut Transform, &mut GridPosition), (With<Monster>, Without<Player>)>,
    mut ai_timer: ResMut<MonsterAiTimer>,
    time: Res<Time>,
    mut game_timer: ResMut<GameTimer>,
) {
    ai_timer.0.tick(time.delta());
    if !ai_timer.0.finished() {
        return;
    }

    let player_pos = match player_query.get_single() {
        Ok(pos) => pos,
        Err(_) => return,
    };
    let (mut monster_transform, mut monster_pos) = match monster_query.get_single_mut() {
        Ok(m) => m,
        Err(_) => return,
    };

    let result = astar(
        &(*monster_pos),
        |p| {
            let mut neighbors = Vec::with_capacity(4);
            let &GridPosition(pos) = p;
            for (dx, dy) in [(0, 1), (0, -1), (1, 0), (-1, 0)].iter() {
                let next_x = pos.x as i32 + dx;
                let next_y = pos.y as i32 + dy;
                if next_x >= 0 && next_x < maze.width as i32 && next_y >= 0 && next_y < maze.height as i32 {
                    let next_grid_pos = GridPosition(UVec2::new(next_x as u32, next_y as u32));
                    if maze.grid[next_grid_pos.y as usize][next_grid_pos.x as usize] != Cell::Wall {
                        neighbors.push((next_grid_pos, 1));
                    }
                }
            }
            neighbors.into_iter()
        },
        |p| (p.x.abs_diff(player_pos.x) + p.y.abs_diff(player_pos.y)),
        |p| *p == *player_pos,
    );

    if let Some((path, _cost)) = result {
        if path.len() > 1 {
            let next_step = path[1];
            
            let new_world_pos = grid_to_world_coordinates(next_step, maze.width, maze.height);
            monster_transform.translation.x = new_world_pos.x;
            monster_transform.translation.y = new_world_pos.y;
            *monster_pos = next_step;

            if *monster_pos == *player_pos {
                println!("\nGAME OVER! The monster got you. Returning to menu...");
                game_timer.0.pause();
                next_state.set(AppState::MainMenu);
            }
        }
    }
}

fn update_timer_ui(game_timer: Res<GameTimer>, mut query: Query<&mut Text, With<TimerText>>) {
    if let Ok(mut text) = query.get_single_mut() {
        text.sections[0].value = format!("Time: {:.2}", game_timer.0.elapsed_secs());
    }
}

fn cleanup_game(mut commands: Commands, query: Query<Entity, With<GameEntity>>) {
    for entity in &query {
        commands.entity(entity).despawn_recursive();
    }
}


// --- 辅助函数 (Helper Functions) ---
fn spawn_level(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    game_config: Res<GameConfig>,
    mut q_windows: Query<&mut Window>,
    mut game_timer: ResMut<GameTimer>,
) {
    game_timer.0.reset();
    game_timer.0.unpause();
    
    let (width, height) = game_config.difficulty.get_dimensions();

    let mut window = q_windows.single_mut();
    window.resolution.set(width as f32 * TILE_SIZE, height as f32 * TILE_SIZE);

    commands.spawn((Camera2dBundle::default(), GameEntity));
    
    let maze = generate_maze_with_start_exit(width, height);

    let tilemap_texture_handle: Handle<Image> = asset_server.load("tiles.png");
    let map_size = TilemapSize { x: width, y: height };
    let tile_size = TilemapTileSize { x: TILE_SIZE, y: TILE_SIZE };
    let grid_size = tile_size.into();
    let map_type = TilemapType::default();
    let mut tile_storage = TileStorage::empty(map_size);
    let tilemap_entity = commands.spawn(GameEntity).id();
    for y in 0..maze.height {
        for x in 0..maze.width {
            let tile_pos = TilePos { x, y };
            let tile_texture_index = match maze.grid[y as usize][x as usize] {
                Cell::Wall => TILE_WALL,
                Cell::Path => TILE_PATH,
                Cell::Start => TILE_START,
                Cell::Exit => TILE_EXIT,
            };
            let tile_entity = commands.spawn((GameEntity, TileBundle { position: tile_pos, tilemap_id: TilemapId(tilemap_entity), texture_index: TileTextureIndex(tile_texture_index), ..default() })).id();
            tile_storage.set(&tile_pos, tile_entity);
        }
    }
    let tilemap_transform = get_tilemap_center_transform(&map_size, &grid_size, &map_type, Z_LAYER_MAZE);
    commands.entity(tilemap_entity).insert(TilemapBundle { grid_size, map_type, size: map_size, storage: tile_storage, texture: TilemapTexture::Single(tilemap_texture_handle), tile_size, transform: tilemap_transform, ..default() });

    let player_texture_handle: Handle<Image> = asset_server.load("player.png");
    let player_world_pos = grid_to_world_coordinates(GridPosition(maze.start_pos), maze.width, maze.height);
    commands.spawn(( GameEntity, Player, GridPosition(maze.start_pos), SpriteBundle { texture: player_texture_handle, transform: Transform::from_xyz(player_world_pos.x, player_world_pos.y, Z_LAYER_PLAYER), ..default() }, ));

    let monster_texture_handle: Handle<Image> = asset_server.load("monster.png");
    let monster_world_pos = grid_to_world_coordinates(GridPosition(maze.monster_start_pos), maze.width, maze.height);
    commands.spawn(( GameEntity, Monster, GridPosition(maze.monster_start_pos), SpriteBundle { texture: monster_texture_handle, transform: Transform::from_xyz(monster_world_pos.x, monster_world_pos.y, Z_LAYER_MONSTER), ..default() }, ));

    commands.spawn(( GameEntity, NodeBundle { style: Style { width: Val::Percent(100.0), height: Val::Percent(100.0), align_items: AlignItems::FlexStart, justify_content: JustifyContent::FlexStart, ..default() }, ..default() }, )).with_children(|parent| {
        parent.spawn(( TimerText, TextBundle::from_section( "Time: 0.00", TextStyle { font_size: 24.0, color: Color::WHITE, ..default() }, ).with_style(Style { margin: UiRect::all(Val::Px(10.0)), ..default() }), ));
    });

    commands.insert_resource(maze);
}

fn grid_to_world_coordinates(grid_pos: GridPosition, maze_width: u32, maze_height: u32) -> Vec2 {
    let map_width_pixels = maze_width as f32 * TILE_SIZE;
    let map_height_pixels = maze_height as f32 * TILE_SIZE;

    let world_x = (grid_pos.x as f32 * TILE_SIZE) - (map_width_pixels / 2.0) + (TILE_SIZE / 2.0);
    let world_y = (grid_pos.y as f32 * TILE_SIZE) - (map_height_pixels / 2.0) + (TILE_SIZE / 2.0);

    Vec2::new(world_x, world_y)
}

pub fn generate_maze_with_start_exit(width: u32, height: u32) -> Maze {
    let mut grid = vec![vec![Cell::Wall; width as usize]; height as usize];
    let mut rng = thread_rng();
    let mut stack = Vec::new();

    let start_node = UVec2::new(1, 1);
    grid[start_node.y as usize][start_node.x as usize] = Cell::Path;
    stack.push(start_node);

    while let Some(current) = stack.last().cloned() {
        let mut neighbors = Vec::new();
        if current.y >= 2 { neighbors.push(UVec2::new(current.x, current.y - 2)); }
        if current.y < height - 2 { neighbors.push(UVec2::new(current.x, current.y + 2)); }
        if current.x >= 2 { neighbors.push(UVec2::new(current.x - 2, current.y)); }
        if current.x < width - 2 { neighbors.push(UVec2::new(current.x + 2, current.y)); }

        let unvisited_neighbors: Vec<_> = neighbors.into_iter()
            .filter(|next| grid[next.y as usize][next.x as usize] == Cell::Wall)
            .collect();

        if !unvisited_neighbors.is_empty() {
            if let Some(next_node) = unvisited_neighbors.choose(&mut rng).cloned() {
                let wall_to_break = UVec2::new((current.x + next_node.x) / 2, (current.y + next_node.y) / 2);
                grid[next_node.y as usize][next_node.x as usize] = Cell::Path;
                grid[wall_to_break.y as usize][wall_to_break.x as usize] = Cell::Path;
                stack.push(next_node);
            }
        } else {
            stack.pop();
        }
    }

    let mut path_cells = Vec::new();
    for y in 0..height {
        for x in 0..width {
            if grid[y as usize][x as usize] == Cell::Path {
                path_cells.push(UVec2::new(x, y));
            }
        }
    }
    
    let (start_pos, exit_pos) = if path_cells.len() >= 20 {
        let maze_center = Vec2::new(width as f32 / 2.0, height as f32 / 2.0);
        path_cells.sort_by(|a, b| {
            let dist_a = a.as_vec2().distance_squared(maze_center);
            let dist_b = b.as_vec2().distance_squared(maze_center);
            dist_a.partial_cmp(&dist_b).unwrap_or(std::cmp::Ordering::Equal)
        });
        let cushion = (path_cells.len() as f32 * 0.15).ceil() as usize;
        let inner_ring = &path_cells[..cushion];
        let outer_ring = &path_cells[path_cells.len() - cushion..];
        let start = *outer_ring.choose(&mut rng).unwrap_or(&path_cells[path_cells.len() - 1]);
        let exit = *inner_ring.choose(&mut rng).unwrap_or(&path_cells[0]);
        (start, exit)
    } else {
        path_cells.shuffle(&mut rng);
        let start = path_cells.pop().unwrap_or(UVec2::new(1, 1));
        let exit = path_cells.pop().unwrap_or(UVec2::new(width - 2, height - 2));
        (start, exit)
    };

    grid[start_pos.y as usize][start_pos.x as usize] = Cell::Start;
    grid[exit_pos.y as usize][exit_pos.x as usize] = Cell::Exit;

    let monster_start_pos = find_most_distant_path(&grid, start_pos, exit_pos);

    Maze { width, height, grid, start_pos, exit_pos, monster_start_pos }
}

fn find_most_distant_path(grid: &Vec<Vec<Cell>>, player_start: UVec2, default_pos: UVec2) -> UVec2 {
    let mut path_cells = Vec::new();
    let height = grid.len() as u32;
    let width = grid[0].len() as u32;

    for y in 1..height - 1 {
        for x in 1..width - 1 {
            if grid[y as usize][x as usize] == Cell::Path {
                path_cells.push(UVec2::new(x, y));
            }
        }
    }
    
    if path_cells.is_empty() {
        return default_pos;
    }

    path_cells.sort_by(|a, b| {
        let dist_a = a.as_vec2().distance_squared(player_start.as_vec2());
        let dist_b = b.as_vec2().distance_squared(player_start.as_vec2());
        dist_b.partial_cmp(&dist_a).unwrap()
    });

    path_cells[0]
}
