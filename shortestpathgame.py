import pygame
import pygame_gui
import sys
import math
import heapq
import random
from shapely.geometry import LineString, Polygon, Point, box

pygame.init()
WINDOW_WIDTH, WINDOW_HEIGHT = 1000, 700
MENU_HEIGHT = 100            
BOTTOM_BAR_HEIGHT = 30       
BUTTON_WIDTH, BUTTON_HEIGHT = 100, 50
BUTTON_MARGIN = 10
FONT_SIZE = 24

WHITE = (255, 255, 255)
DARK_GRAY = (50, 50, 50)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
PATH_COLOR = (255, 165, 0)

TOOLS = ['Start', 'Reset', 'Stop',
         'Triangle', 'Square', 'Circle', 'Line',
         'A', 'B']
screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
pygame.display.set_caption("CG Project (Shortest path among obstacles)")

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

clock = pygame.time.Clock()
obstacles_shapely = []
point_A = None
point_B = None
path = [] 

current_tool = None
placing_shape = False

mouse_pos = (0,0)
preview_angle = 0.0 
preview_scale = 1.0 
MIN_SCALE = 0.1
MAX_SCALE = 10.0
BASE_SIZE = {
    'Triangle': 50,
    'Square': 50,
    'Circle': 25,
    'Line': 60, 
}
LINE_THICKNESS = 4
def make_triangle(size=50):
    half = size / 2
    pts = [
        (0,     -half),
        ( half,  half),
        (-half,  half),
    ]
    return pts

def make_square(size=50):
    half = size / 2
    pts = [
        (-half, -half),
        ( half, -half),
        ( half,  half),
        (-half,  half),
    ]
    return pts

def make_circle(radius=25, segments=16):
    pts = []
    for i in range(segments):
        ang = 2*math.pi*i/segments
        x = radius * math.cos(ang)
        y = radius * math.sin(ang)
        pts.append((x,y))
    return pts

def make_line(length=60, thickness=4):
    half_len = length/2
    half_thk = thickness/2
    pts = [
        (-half_len, -half_thk),
        ( half_len, -half_thk),
        ( half_len,  half_thk),
        (-half_len,  half_thk),
    ]
    return pts
def transform_shape(base_points, cx, cy, scale, angle_deg):
    rad = math.radians(angle_deg)
    cosA = math.cos(rad)
    sinA = math.sin(rad)
    out = []
    for (bx, by) in base_points:

        sx = bx * scale
        sy = by * scale

        rx = sx*cosA - sy*sinA
        ry = sx*sinA + sy*cosA

        fx = rx + cx
        fy = ry + cy
        out.append((fx, fy))
    return out
def reset_playground():
    global obstacles_shapely, point_A, point_B, path
    global preview_angle, preview_scale
    obstacles_shapely = []

    top_bar_poly = box(0, -100, WINDOW_WIDTH, 0)
    obstacles_shapely.append(top_bar_poly)

    bottom_y1 = WINDOW_HEIGHT - BOTTOM_BAR_HEIGHT
    bottom_bar_poly = box(0, bottom_y1 - MENU_HEIGHT, WINDOW_WIDTH, WINDOW_HEIGHT - MENU_HEIGHT)
    obstacles_shapely.append(bottom_bar_poly)

    point_A = None
    point_B = None
    path = []
    preview_angle = 0.0
    preview_scale = 1.0

def stop_path():
    global path
    path = []
def compute_shortest_path(start, goal, polygons_shapely):
    if not start or not goal:
        print("No start or goal.")
        return []
    nodes = [start, goal]
    for poly in polygons_shapely:
        coords = list(poly.exterior.coords)
        if len(coords)>1:
            nodes.extend(coords[:-1])
    vis_graph = build_visibility_graph(nodes, polygons_shapely)
    path_found = dijkstra(vis_graph, start, goal)
    return path_found

def build_visibility_graph(nodes, polygons):
    visibility_graph = {node: [] for node in nodes}
    for i, origin in enumerate(nodes):
        visible_nodes = compute_visible_nodes(origin, nodes, polygons)
        for other in visible_nodes:
            if other != origin:
                line = LineString([origin, other])
                dist = line.length
                visibility_graph[origin].append((other, dist))
    return visibility_graph

def compute_visible_nodes(origin, nodes, polygons):
    visible = set()
    for target in nodes:
        if target == origin:
            continue
        seg = LineString([origin, target])
        if not is_line_blocked(seg, polygons):
            visible.add(target)
    return visible

def is_line_blocked(line_seg, polygons):
    for poly in polygons:

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
def main():
    global current_tool, placing_shape
    global mouse_pos, preview_angle, preview_scale
    global obstacles_shapely, point_A, point_B, path

    reset_playground()

    running = True
    while running:
        dt = clock.tick(60) / 1000.0


        keys = pygame.key.get_pressed()
        if keys[pygame.K_r]:
            preview_angle += 60 * dt
            preview_angle %= 360

        for event in pygame.event.get():
            manager.process_events(event)

            if event.type == pygame.QUIT:
                running = False
                break

    
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
                        print("Compute path...")
                        path_result = compute_shortest_path(point_A, point_B, obstacles_shapely)
                        if path_result:
                            path = path_result
                            print("Path found.")
                        else:
                            path = []
                            print("No path found.")
                    elif event.ui_element == buttons['Reset']:
                        reset_playground()
                        print("Reset done.")
                    elif event.ui_element == buttons['Stop']:
                        stop_path()
                        print("Stop path.")
                    elif event.ui_element == buttons['Triangle']:
                        current_tool = 'Triangle'
                        placing_shape = True
                    elif event.ui_element == buttons['Square']:
                        current_tool = 'Square'
                        placing_shape = True
                    elif event.ui_element == buttons['Circle']:
                        current_tool = 'Circle'
                        placing_shape = True
                    elif event.ui_element == buttons['Line']:
                        current_tool = 'Line'
                        placing_shape = True
                    elif event.ui_element == buttons['A']:
                        current_tool = 'A'
                        placing_shape = False
                    elif event.ui_element == buttons['B']:
                        current_tool = 'B'
                        placing_shape = False

    
            if event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:
                    mx, my = event.pos
            
            
                    if my < MENU_HEIGHT or my >= WINDOW_HEIGHT - BOTTOM_BAR_HEIGHT:
                
                        continue

            
                    if current_tool in ['Triangle','Square','Circle','Line']:
                        local_x = mx
                        local_y = my - MENU_HEIGHT

                        if current_tool == 'Triangle':
                            base_pts = make_triangle(BASE_SIZE['Triangle'])
                        elif current_tool == 'Square':
                            base_pts = make_square(BASE_SIZE['Square'])
                        elif current_tool == 'Circle':
                            base_pts = make_circle(BASE_SIZE['Circle'], 16)
                        elif current_tool == 'Line':
                            base_pts = make_line(BASE_SIZE['Line'], LINE_THICKNESS)

                        final_pts = transform_shape(base_pts, local_x, local_y,
                                                    preview_scale, preview_angle)
                        new_poly = Polygon(final_pts)
                        obstacles_shapely.append(new_poly)

                        placing_shape = False
                        current_tool = None

            
                    elif current_tool == 'A':
                        point_A = (float(mx), float(my - MENU_HEIGHT))
                        print("Set A:", point_A)
                        current_tool = None
                    elif current_tool == 'B':
                        point_B = (float(mx), float(my - MENU_HEIGHT))
                        print("Set B:", point_B)
                        current_tool = None

    
            if event.type == pygame.MOUSEMOTION:
                mx, my = event.pos
        
                if my >= MENU_HEIGHT and my < WINDOW_HEIGHT - BOTTOM_BAR_HEIGHT:
                    mouse_pos = (mx, my - MENU_HEIGHT)



        screen.fill(WHITE)




        for poly in obstacles_shapely:
            coords = list(poly.exterior.coords)
            if len(coords) > 1:
                draw_pts = [(p[0], p[1] + MENU_HEIGHT) for p in coords]
                pygame.draw.polygon(screen, BLACK, draw_pts, width=2)


        if point_A:
            pxA, pyA = point_A
            pygame.draw.circle(screen, GREEN, (int(pxA), int(pyA + MENU_HEIGHT)), 5)
        if point_B:
            pxB, pyB = point_B
            pygame.draw.circle(screen, RED, (int(pxB), int(pyB + MENU_HEIGHT)), 5)


        if len(path) > 1:
            shifted_path = [(p[0], p[1] + MENU_HEIGHT) for p in path]
            pygame.draw.lines(screen, PATH_COLOR, False, shifted_path, 3)


        if placing_shape and current_tool in ['Triangle','Square','Circle','Line']:
            (cx, cy) = mouse_pos
            if current_tool == 'Triangle':
                base_pts = make_triangle(BASE_SIZE['Triangle'])
            elif current_tool == 'Square':
                base_pts = make_square(BASE_SIZE['Square'])
            elif current_tool == 'Circle':
                base_pts = make_circle(BASE_SIZE['Circle'], 16)
            elif current_tool == 'Line':
                base_pts = make_line(BASE_SIZE['Line'], LINE_THICKNESS)

            preview_pts = transform_shape(base_pts, cx, cy, preview_scale, preview_angle)
            draw_pts = [(x, y + MENU_HEIGHT) for (x,y) in preview_pts]
            pygame.draw.polygon(screen, (0,0,0,80), draw_pts, width=2)


        pygame.draw.rect(screen, DARK_GRAY, (0, 0, WINDOW_WIDTH, MENU_HEIGHT))


        pygame.draw.rect(screen, DARK_GRAY,
                         (0, WINDOW_HEIGHT - BOTTOM_BAR_HEIGHT,
                          WINDOW_WIDTH, BOTTOM_BAR_HEIGHT))


        manager.update(dt)
        manager.draw_ui(screen)

        pygame.display.flip()

    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()
