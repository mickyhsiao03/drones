from stanfordkarel import *
from stanfordkarel import karel_program
from datetime import datetime

target = [(2,2), (3,5), (4,7), (7,7), (6,1), (4,5)]
size = (7,7)

# target = [(6,39), (31,15), (32,27), (8,33), (9,18), (4,5),(22,2), (13,15), (40,17), (27,37), (6,15), (14,5), (18,26), (17,16), (19,17), (23,27), (26,15), (14,25)]
# size = (40,40)

droneCommands = []

def turn_back():
    turn_left()
    turn_left()

def turn_right():
    turn_left()
    turn_left()
    turn_left()

#movement controls, goes to correct space no matter which way it is facing
def move_top_right():
    if facing_north():
        move()
        turn_right()
        move()
        droneCommands.extend(("forward 20", "right 20"))
    elif facing_east():
        move()
        turn_left()
        move()
        droneCommands.extend(("foward 20", "left 20"))
    elif facing_south():
        turn_left()
        move()
        turn_left()
        move()
        droneCommands.extend(("left 20", "backward 20"))
    elif facing_west():
        turn_right()
        move()
        turn_right()
        move()
        droneCommands.extend(("right 20", "backward 20"))
def move_right():
    if facing_north():
        turn_right()
        move()
        droneCommands.append(("right 20"))
    elif facing_east():
        move()   
        droneCommands.append(("foward 20"))
    elif facing_south():
        turn_left()
        move()
        droneCommands.append(("left 20"))
    elif facing_west():
        turn_back()
        move()
        droneCommands.append(("backward 20"))
def move_bot_right():
    if facing_north():
        turn_back()
        move()
        turn_left()
        move()
        droneCommands.extend(("backward 20", "right 20"))
    elif facing_east():
        turn_right()
        move()
        turn_left()
        move()
        droneCommands.extend(("right 20", "forward 20"))
    elif facing_south():
        move()
        turn_left()
        move()
        droneCommands.extend(("foward 20", "left 20"))
    elif facing_west():
        turn_right()
        move()
        turn_right()
        move()
        droneCommands.extend(("left 20", "backward 20"))
def move_bot():
    if facing_north():
        turn_back()
        move()
        droneCommands.append(("backward 20"))
    elif facing_east():
        turn_right()
        move() 
        droneCommands.append(("right 20"))
    elif facing_south():
        move()
        droneCommands.append(("foward 20"))
    elif facing_west():
        turn_left()
        move()
        droneCommands.append(("left 20"))
def move_bot_left():
    if facing_north():
        turn_back()
        move()
        turn_right()
        move()
        droneCommands.extend(("backward 20", "left 20"))
    elif facing_east():
        turn_right()
        move()
        turn_right()
        move()
        droneCommands.extend(("right 20", "backward 20"))
    elif facing_south():
        move()
        turn_right()
        move()
        droneCommands.extend(("foward 20", "right 20"))
    elif facing_west():
        turn_left()
        move()
        turn_right()
        move()
        droneCommands.extend(("left 20", "forward 20"))
def move_left():
    if facing_north():
        turn_left()
        move()
        droneCommands.append(("left 20"))
    elif facing_east():
        turn_back()
        move() 
        droneCommands.append(("backward 20"))
    elif facing_south():
        turn_right()
        move()
        droneCommands.append(("right 20"))
    elif facing_west():
        move()
        droneCommands.append(("foward 20"))
def move_top_left():    
    if facing_north():
        move()
        turn_left()
        move()
        droneCommands.extend(("foward 20", "left 20"))
    elif facing_east():
        turn_left()
        move()
        turn_left()
        move()   
        droneCommands.extend(("left 20", "backward 20"))
    elif facing_south():
        turn_back()
        move()
        turn_left()
        move()
        droneCommands.extend(("backward x", "right x"))
    elif facing_west():
        turn_right()
        move()
        turn_left()
        move()
        droneCommands.extend(("right 20", "forward 20"))
def move_top():   
    if facing_north():
        move()
        droneCommands.append(("foward 20"))
    elif facing_east():
        turn_left()
        move()    
        droneCommands.append(("left 20"))
    elif facing_south():
        turn_back()
        move()
        droneCommands.append(("backward 20"))
    elif facing_west():
        turn_right()
        move()
        droneCommands.append(("right 20"))

def getClose(targets, curlo): #returns the closest target node
    nextlist={}
    diag = 14
    flat = 10

    for potential in targets:
        h_cost= heuristic(curlo, diag, flat, potential)
        nextlist[potential] = h_cost

    sorted_values = sorted(nextlist.values())
    sorted_dict = {}
    for i in sorted_values:
        for k in nextlist.keys():
            if nextlist[k] == i:
                sorted_dict[k] = nextlist[k]

    result = next(iter(sorted_dict))

    return result
        
def heuristic(curlo, diag, flat, closest): #find h cost, distance from goal node
    dx = abs(curlo[0] - closest[0])
    dy = abs(curlo[1] - closest[1])
    return flat * (dx + dy) + (diag - 2 * flat) * min(dx, dy)

def calcg_cost(curlo, diag, flat, nextStep): #find g cost, distance from starting node
    x = nextStep[0] - curlo[0]
    y = nextStep[1] - curlo[1]

    if (x,y) == (1,1) or (x,y) == (-1,-1) or (x,y) == (-1,1) or (x,y) == (1,-1):
        return diag
    if (x,y) == (1,0) or (x,y) == (0,1) or (x,y) == (-1,0) or (x,y) == (0,-1):
        return flat

def findNext(curlo, closest): #returns list of potential steps to take (neighbours)
    
    openList = []

    openList.append(curlo)

    neigh = [] 
    
    if (curlo[0] + 1) > 0 and  (curlo[1] + 1) > 0:#top right 
        neigh.append(((curlo[0] + 1, curlo[1] +1)))

    if (curlo[0] - 1) > 0 and (curlo[1] + 1) > 0:#top left 
        neigh.append(((curlo[0] - 1, curlo[1] +1))) 
    
    if (curlo[0] - 1) > 0 and (curlo[1] - 1) > 0:#bottom left
        neigh.append(((curlo[0] - 1, curlo[1] -1))) 

    if (curlo[0] + 1) > 0 and (curlo[1] - 1) > 0:#bottom right
        neigh.append(((curlo[0] + 1, curlo[1] -1))) 

    if (curlo[0] + 1) > 0 and (curlo[1]) > 0:#right
        neigh.append(((curlo[0] + 1, curlo[1]))) 

    if (curlo[0]) > 0 and (curlo[1] + 1) > 0:#top
        neigh.append(((curlo[0], curlo[1] + 1))) 
    
    if (curlo[0]) > 0 and (curlo[1] - 1) > 0:#bottom
        neigh.append(((curlo[0], curlo[1] - 1) )) 

    if (curlo[0] - 1) > 0 and (curlo[1]) > 0:#left
        neigh.append(((curlo[0] - 1, curlo[1]))) 

    return neigh

def getBest(potentialStep, diag, flat, curlo, closest): #finds lowest f_cost, which is best choice to move
    nextlist = {}

    for i in potentialStep:
        h_cost= heuristic(i, diag, flat, closest)
        g_cost= calcg_cost(curlo, diag, flat, i)
        f_cost= h_cost + g_cost
        nextlist[i] = f_cost

    sorted_values = sorted(nextlist.values())
    sorted_dict = {}
    for i in sorted_values:
        for k in nextlist.keys():
            if nextlist[k] == i:
                sorted_dict[k] = nextlist[k]

    result = next(iter(sorted_dict))

    return result

def mission(routeRdy):      #reads each coorinates from the map provided by main function, and travel to them
    curlo = karel_program.location
    print(curlo)
    for eachStep in routeRdy:

        togo = (eachStep[0] - curlo[0], eachStep[1] - curlo[1])
        
        if togo == (0,1):   #determines which way to go based on togo value (x,y)
            move_top()
            if beepers_present():
                droneCommands.append("delay 10")
                paint_corner(GREEN)
            else:
                paint_corner(RED)
        if togo == (1,0):
            move_right()
            if beepers_present():
                droneCommands.append("delay 10")
                paint_corner(GREEN)
            else:
                paint_corner(RED)
        if togo == (1,1):
            move_top_right()
            if beepers_present():
                droneCommands.append("delay 10")
                paint_corner(GREEN)
            else:
                paint_corner(RED)
        if togo == (0,-1):
            move_bot()
            if beepers_present():
                droneCommands.append("delay 10")
                paint_corner(GREEN)
            else:
                paint_corner(RED)
        if togo == (-1,0):
            move_left()
            if beepers_present():
                droneCommands.append("delay 10")
                paint_corner(GREEN)
            else:
                paint_corner(RED)
        if togo == (-1,-1):
            move_bot_left()
            if beepers_present():
                droneCommands.append("delay 10")
                paint_corner(GREEN)
            else:
                paint_corner(RED)
        if togo == (-1,1):
            move_top_left()
            if beepers_present():
                droneCommands.append("delay 10")
                paint_corner(GREEN)
            else:
                paint_corner(RED)
        if togo == (1,-1):
            move_bot_right()
            if beepers_present():
                droneCommands.append("delay 10")
                paint_corner(GREEN)
            else:
                paint_corner(RED)

        curlo = eachStep
        

def main():     #main function, maps out a route for drone to start mission
    close = []
    diag = 14
    flat = 10
    routeRdy = []
    goingHome = False

    beepers_in_bag() #find current location
    curlo = karel_program.location #default (1,1)
    startingPt = curlo
    

    for eachTarget in range(len(target)+1):
 
        if len(target) == 0:
            print('going home')
            closest = startingPt
            goingHome = True

        else:    
            closest = getClose(target, curlo) #find closest target 
            toDel = closest
        
        if goingHome == True:
            while curlo != closest and (curlo[0],curlo[1]) <= (size[0],size[1]):
                nextStep = findNext(curlo, closest) #return list of potential steps
                
                close.append(curlo)
                best = getBest(nextStep,diag,flat,curlo,closest) #finds lowest f_cost (best way to go)
                curlo = best
                print('this is best choice to go',best)  

                if closest in nextStep:
                    close.append(curlo)
                    curlo = closest
                    routeRdy = close
                    break

        else:
            while curlo != closest and (curlo[0],curlo[1]) <= (size[0],size[1]):
                print('im at', curlo)
                
                nextStep = findNext(curlo, closest) #return list of potential steps
                
                close.append(curlo)
                best = getBest(nextStep,diag,flat,curlo,closest) #finds lowest f_cost (best way to go)
                curlo = best
                print('this is best choice to go',best)

                if closest in nextStep:
                    close.append(curlo)
                    curlo = closest
                    print('reached',closest)
                    print('visited', close)
                    routeRdy = close
                    break
        
        if closest == startingPt:
            print('returned home at', curlo)
            goingHome = False
        else:
            target.remove(toDel)
            print('left to go', target)

    #ended route calculation (planning)

    mission(routeRdy) #start mission (drone movement)
    
    now = datetime.now()
    curTime = now.strftime("%H%M%S")
    with open(f"C:/Command.txt", "w") as f: #writes tello command file with time stamp. command.txt official file name
        f.write('command')
        f.write('\n')
        f.write('delay 2')
        f.write('\n')
        f.write('takeoff')
        f.write('\n')
        f.write('delay 2')
        f.write('\n')
        for line in droneCommands:
            f.write(line)
            f.write('\n')
            f.write('delay 2') #delay used to give cpu time to send and receive data
            f.write('\n')
        f.write('land')
    
            

if __name__ == "__main__":
    # run_karel_program("40x40")
    run_karel_program("7x7")


