import pygame
import heapq

# Initialize Pygame
pygame.init()
window = pygame.display.set_mode((1200, 400))
track = pygame.image.load('smart cruise/track6.png')
car = pygame.image.load('smart cruise/tesla.png')
car = pygame.transform.scale(car, (30, 60))

# Car and camera settings
car_x = 155
car_y = 300
focal_dis = 25
cam_x_offset = 0
cam_y_offset = 0
direction = 'up'
drive = True
clock = pygame.time.Clock()

# Fuel settings
fuel = 100
fuel_consumption_rate = 0.1

# A* pathfinding settings
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def a_star_search(start, goal, grid):
    neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0)]
    close_set = set()
    came_from = {}
    gscore = {start: 0}
    fscore = {start: heuristic(start, goal)}
    oheap = []
    heapq.heappush(oheap, (fscore[start], start))
    
    while oheap:
        current = heapq.heappop(oheap)[1]
        
        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            return data
        
        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            tentative_g_score = gscore[current] + heuristic(current, neighbor)
            if 0 <= neighbor[0] < grid.shape[0]:
                if 0 <= neighbor[1] < grid.shape[1]:
                    if grid[neighbor[0]][neighbor[1]] == 1:
                        continue
                else:
                    continue
            else:
                continue
                
            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue
                
            if tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1] for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(oheap, (fscore[neighbor], neighbor))
    
    return False

def detect_obstacle(window, x, y, offset_x=0, offset_y=0, focal_dis=0):
    try:
        return window.get_at((x + offset_x, y + offset_y))[0] != 255
    except IndexError:
        return True

# Main driving loop
while drive:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            drive = False

    clock.tick(60)
    cam_x = car_x + cam_x_offset + 15
    cam_y = car_y + cam_y_offset + 15

    # Check pixel colors around the car
    up_px = detect_obstacle(window, cam_x, cam_y, offset_y=-focal_dis)
    down_px = detect_obstacle(window, cam_x, cam_y, offset_y=focal_dis)
    right_px = detect_obstacle(window, cam_x, cam_y, offset_x=focal_dis)

    # Change direction based on pixel colors (taking turns)
    if direction == 'up' and up_px and not right_px:
        direction = 'right'
        cam_x_offset = 30
        car = pygame.transform.rotate(car, -90)
    elif direction == 'right' and right_px and not down_px:
        direction = 'down'
        car_x += 30
        cam_x_offset = 0
        cam_y_offset = 30
        car = pygame.transform.rotate(car, -90)
    elif direction == 'down' and down_px and not right_px:
        direction = 'right'
        car_y += 30
        cam_x_offset = 30
        cam_y_offset = 0
        car = pygame.transform.rotate(car, 90)
    elif direction == 'right' and right_px and not up_px:
        direction = 'up'
        car_x += 30
        cam_x_offset = 0
        car = pygame.transform.rotate(car, 90)

    # Drive forward in the current direction
    if direction == 'up' and not up_px:
        car_y -= 2
    elif direction == 'right' and not right_px:
        car_x += 2
    elif direction == 'down' and not down_px:
        car_y += 2

    # Consume fuel
    fuel -= fuel_consumption_rate
    if fuel <= 0:
        drive = False
        print("Out of fuel!")
    
    # Render the track and car
    window.blit(track, (0, 0))
    window.blit(car, (car_x, car_y))
    pygame.draw.circle(window, (0, 255, 0), (cam_x, cam_y), 5, 5)
    
    # Render fuel level
    font = pygame.font.Font(None, 36)
    fuel_text = font.render(f'Fuel: {int(fuel)}%', True, (255, 255, 255))
    window.blit(fuel_text, (10, 10))

    # Update the display
    pygame.display.update()
    
pygame.quit()
