import pygame
import pygame_gui
import sys
import math
import heapq
import random

# Shapely for geometry
from shapely.geometry import LineString, Polygon, Point

pygame.init()

# --------------------------------------------------------------------------------
#                                  CONSTANTS
# --------------------------------------------------------------------------------
WINDOW_WIDTH, WINDOW_HEIGHT = 1000, 700
MENU_HEIGHT = 100
BUTTON_WIDTH, BUTTON_HEIGHT = 100, 50
BUTTON_MARGIN = 10
FONT_SIZE = 24

WHITE = (255, 255, 255)
DARK_GRAY = (50, 50, 50)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
PATH_COLOR = (255, 165, 0)  # Orange

TOOLS = ['Start', 'Reset', 'Stop',
         'Triangle', 'Square', 'Circle', 'Line',
         'A', 'B']

# Pygame GUI setup
screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
pygame.display.set_caption("Visibility Graph Path + Rotation + Scaling")

manager = pygame_gui.UIManager((WINDOW_WIDTH, WINDOW_HEIGHT))

def calc_button_positions():
    rects = {}
    x = BUTTON_MARGIN
    y = (MENU_HEIGHT - BUTTON_HEIGHT) // 2
    for tool in TOOLS:
        rects[tool] = pygame.Rect(x, y, BUTTON_WIDTH, BUTTON_HEIGHT)
        x += BUTTON_WIDTH + BUTTON_MARGIN
    return rects

button_rects = calc_button_positions()
buttons = {}
for tool in TOOLS:
    buttons[tool] = pygame_gui.elements.UIButton(
        relative_rect=button_rects[tool],
        text=tool,
        manager=manager
    )

# Surfaces for drawing
playground = pygame.Surface((WINDOW_WIDTH, WINDOW_HEIGHT - MENU_HEIGHT))
playground.fill(WHITE)
preview_surf = pygame.Surface(playground.get_size(), pygame.SRCALPHA)

clock = pygame.time.Clock()

# --------------------------------------------------------------------------------
#    DATA STRUCTURES
# --------------------------------------------------------------------------------
obstacles_shapely = []  # list of shapely Polygons for obstacles

point_A = None
point_B = None
path = []   # final path of (x,y) from A to B

current_tool = None
placing_shape = False

mouse_pos = (0,0)

# For rotation & scaling
preview_angle = 0.0      # in degrees
preview_scale = 1.0      # scale factor (1.0 = 100%)

# shape base sizes (for each tool)
BASE_SIZE = {
    'Triangle': 50,
    'Square': 50,
    'Circle': 25,
    'Line': 60,   # line length
}
LINE_THICKNESS = 4

# Limits to avoid zero or negative scale
MIN_SCALE = 0.1
MAX_SCALE = 10.0

# --------------------------------------------------------------------------------
#   SHAPE BUILDERS (UNTRANSFORMED)
# --------------------------------------------------------------------------------

def make_triangle(size=50):
    """
    Base upright triangle around (0,0). 
    We'll later rotate & shift it around the preview center.
    The bounding radius is 'size/2' up/down from origin.
    """
    half = size / 2
    pts = [
        (0, -half),        # top
        ( half, half),     # bottom right
        (-half, half),     # bottom left
    ]
    return pts

def make_square(size=50):
    """
    Centered at (0,0).
    """
    half = size / 2
    pts = [
        (-half, -half),
        ( half, -half),
        ( half,  half),
        (-half,  half),
    ]
    return pts

def make_circle(radius=25, segments=16):
    """
    A polygon approximation of a circle around (0,0).
    Rotation won't matter visually, but we still apply it for consistency.
    """
    pts = []
    for i in range(segments):
        ang = 2*math.pi*i/segments
        x = radius * math.cos(ang)
        y = radius * math.sin(ang)
        pts.append((x,y))
    return pts

def make_line(length=60, thickness=4):
    """
    Horizontal rectangle around (0,0). 
    'length' wide, 'thickness' tall, centered at origin.
    """
    half_len = length / 2
    half_thk = thickness / 2
    pts = [
        (-half_len, -half_thk),
        ( half_len, -half_thk),
        ( half_len,  half_thk),
        (-half_len,  half_thk),
    ]
    return pts

# --------------------------------------------------------------------------------
#  APPLY SCALE & ROTATION & SHIFT
# --------------------------------------------------------------------------------
def transform_shape(base_points, cx, cy, scale, angle_deg):
    """
    1) Scale around (0,0) by 'scale'
    2) Rotate around (0,0) by angle_deg
    3) Translate to (cx, cy)
    Returns list of (x, y)
    """
    rad = math.radians(angle_deg)
    cosA = math.cos(rad)
    sinA = math.sin(rad)
    out = []
    for (bx, by) in base_points:
        # scale
        sx = bx * scale
        sy = by * scale
        # rotate
        rx = sx*cosA - sy*sinA
        ry = sx*sinA + sy*cosA
        # translate
        fx = rx + cx
        fy = ry + cy
        out.append((fx, fy))
    return out

# --------------------------------------------------------------------------------
#   RESET / STOP
# --------------------------------------------------------------------------------
def reset_playground():
    global obstacles_shapely, point_A, point_B, path
    global preview_angle, preview_scale
    obstacles_shapely = []
    point_A = None
    point_B = None
    path = []
    # Reset transform to defaults
    preview_angle = 0.0
    preview_scale = 1.0
    playground.fill(WHITE)

def stop_path():
    global path
    path = []

# --------------------------------------------------------------------------------
#   VISIBILITY GRAPH PATHFINDING (same as before)
# --------------------------------------------------------------------------------

def compute_shortest_path(start, goal, polygons_shapely):
    if not start or not goal:
        print("No start or goal.")
        return []

    # Build node list
    nodes = [start, goal]
    for poly in polygons_shapely:
        coords = list(poly.exterior.coords)
        if len(coords)>1:
            nodes.extend(coords[:-1])

    # Build graph
    vis_graph = build_visibility_graph(nodes, polygons_shapely)

    # Dijkstra
    path_found = dijkstra(vis_graph, start, goal)
    return path_found

def build_visibility_graph(nodes, polygons_shapely):
    visibility_graph = {node: [] for node in nodes}
    for i, node in enumerate(nodes):
        visible_nodes = compute_visible_nodes(node, nodes, polygons_shapely)
        for other in visible_nodes:
            if other != node:
                line = LineString([node, other])
                dist = line.length
                visibility_graph[node].append((other, dist))
    return visibility_graph

def compute_visible_nodes(origin, nodes, polygons_shapely):
    visible = set()
    for target in nodes:
        if target == origin:
            continue
        seg = LineString([origin, target])
        if not is_line_blocked(seg, polygons_shapely):
            visible.add(target)
    return visible

def is_line_blocked(line_seg, polygons_shapely):
    for poly in polygons_shapely:
        if line_seg.crosses(poly):
            return True
        if line_seg.within(poly):
            return True
    return False

def dijkstra(graph, start, goal):
    if start not in graph or goal not in graph:
        return []
    queue = []
    heapq.heappush(queue, (0, start))
    dist = {start: 0}
    prev = {start: None}

    while queue:
        cur_dist, cur_node = heapq.heappop(queue)
        if cur_node == goal:
            # reconstruct
            path = []
            while cur_node is not None:
                path.append(cur_node)
                cur_node = prev[cur_node]
            path.reverse()
            return path
        if cur_dist > dist[cur_node]:
            continue
        for (nbr, w) in graph[cur_node]:
            alt = cur_dist + w
            if nbr not in dist or alt < dist[nbr]:
                dist[nbr] = alt
                prev[nbr] = cur_node
                heapq.heappush(queue, (alt, nbr))
    return []

# --------------------------------------------------------------------------------
#                              MAIN LOOP
# --------------------------------------------------------------------------------
running = True
while running:
    dt = clock.tick(60) / 1000.0
    preview_surf.fill((0,0,0,0))

    # Check keys each frame for rotation
    keys = pygame.key.get_pressed()
    if keys[pygame.K_r]:
        # E.g. rotate 60 degrees per second
        preview_angle += 60 * dt
        preview_angle %= 360  # keep it 0..359

    for event in pygame.event.get():
        # Let pygame_gui handle events
        manager.process_events(event)

        if event.type == pygame.QUIT:
            running = False

        # Scale on keydown (arrow up/down)
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_UP:
                preview_scale *= 1.05
                if preview_scale > MAX_SCALE:
                    preview_scale = MAX_SCALE
            elif event.key == pygame.K_DOWN:
                preview_scale /= 1.05
                if preview_scale < MIN_SCALE:
                    preview_scale = MIN_SCALE

        if event.type == pygame.USEREVENT:
            if event.user_type == pygame_gui.UI_BUTTON_PRESSED:
                if event.ui_element == buttons['Start']:
                    print("Running visibility-graph pathfinding...")
                    path_result = compute_shortest_path(point_A, point_B, obstacles_shapely)
                    if path_result:
                        path = path_result
                        print("Path found:", path)
                    else:
                        path = []
                        print("No path found.")
                elif event.ui_element == buttons['Reset']:
                    reset_playground()
                    print("Reset complete.")
                elif event.ui_element == buttons['Stop']:
                    stop_path()
                    print("Stop path.")
                elif event.ui_element == buttons['Triangle']:
                    current_tool = 'Triangle'
                    placing_shape = True
                    print("Triangle selected.")
                elif event.ui_element == buttons['Square']:
                    current_tool = 'Square'
                    placing_shape = True
                    print("Square selected.")
                elif event.ui_element == buttons['Circle']:
                    current_tool = 'Circle'
                    placing_shape = True
                    print("Circle selected.")
                elif event.ui_element == buttons['Line']:
                    current_tool = 'Line'
                    placing_shape = True
                    print("Line selected.")
                elif event.ui_element == buttons['A']:
                    current_tool = 'A'
                    placing_shape = False
                    print("Next click => point A.")
                elif event.ui_element == buttons['B']:
                    current_tool = 'B'
                    placing_shape = False
                    print("Next click => point B.")

        if event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1:  # left click
                mx, my = event.pos
                if my >= MENU_HEIGHT:
                    if current_tool in ['Triangle','Square','Circle','Line']:
                        # place shape
                        local_x = mx
                        local_y = my - MENU_HEIGHT
                        # build base shape, apply transform
                        if current_tool == 'Triangle':
                            base_pts = make_triangle(BASE_SIZE['Triangle'])
                        elif current_tool == 'Square':
                            base_pts = make_square(BASE_SIZE['Square'])
                        elif current_tool == 'Circle':
                            base_pts = make_circle(BASE_SIZE['Circle'], segments=16)
                        elif current_tool == 'Line':
                            base_pts = make_line(BASE_SIZE['Line'], LINE_THICKNESS)

                        # apply user scale & rotation
                        final_pts = transform_shape(base_pts, local_x, local_y,
                                                    preview_scale, preview_angle)
                        # create polygon
                        new_poly = Polygon(final_pts)
                        obstacles_shapely.append(new_poly)

                        # done placing
                        placing_shape = False
                        current_tool = None

                    elif current_tool == 'A':
                        point_A = (float(mx), float(my - MENU_HEIGHT))
                        print("Point A set to", point_A)
                        current_tool = None
                    elif current_tool == 'B':
                        point_B = (float(mx), float(my - MENU_HEIGHT))
                        print("Point B set to", point_B)
                        current_tool = None

        if event.type == pygame.MOUSEMOTION:
            mx, my = event.pos
            if my >= MENU_HEIGHT:
                mouse_pos = (mx - 0, my - MENU_HEIGHT)

    # Preview logic
    if placing_shape and current_tool in ['Triangle','Square','Circle','Line']:
        (cx, cy) = mouse_pos
        # Build the base shape
        if current_tool == 'Triangle':
            base_pts = make_triangle(BASE_SIZE['Triangle'])
        elif current_tool == 'Square':
            base_pts = make_square(BASE_SIZE['Square'])
        elif current_tool == 'Circle':
            base_pts = make_circle(BASE_SIZE['Circle'], 16)
        elif current_tool == 'Line':
            base_pts = make_line(BASE_SIZE['Line'], LINE_THICKNESS)
        # Transform
        preview_pts = transform_shape(base_pts, cx, cy, preview_scale, preview_angle)
        # Draw outline
        pygame.draw.polygon(preview_surf, (0,0,0,80), preview_pts, width=2)

    # Update GUI
    manager.update(dt)

    # 1) Draw top bar
    screen.fill(WHITE)
    pygame.draw.rect(screen, DARK_GRAY, (0, 0, WINDOW_WIDTH, MENU_HEIGHT))

    # 2) Draw GUI
    manager.draw_ui(screen)

    # 3) Draw playground
    screen.blit(playground, (0, MENU_HEIGHT))

    # 4) Draw existing obstacles
    for poly in obstacles_shapely:
        coords = list(poly.exterior.coords)
        if len(coords)>1:
            draw_pts = [(p[0], p[1] + MENU_HEIGHT) for p in coords]
            pygame.draw.polygon(screen, BLACK, draw_pts, width=2)

    # 5) Draw A & B
    if point_A:
        pxA, pyA = point_A
        pygame.draw.circle(screen, GREEN, (int(pxA), int(pyA + MENU_HEIGHT)), 5)
    if point_B:
        pxB, pyB = point_B
        pygame.draw.circle(screen, RED, (int(pxB), int(pyB + MENU_HEIGHT)), 5)

    # 6) Draw path
    if len(path) > 1:
        shifted_path = [(p[0], p[1] + MENU_HEIGHT) for p in path]
        pygame.draw.lines(screen, PATH_COLOR, False, shifted_path, 3)

    # 7) Blit preview
    screen.blit(preview_surf, (0, MENU_HEIGHT))

    pygame.display.flip()

pygame.quit()
sys.exit()
