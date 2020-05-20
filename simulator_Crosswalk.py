# -*-coding:utf-8-*-
# Author: Scott Larter

import pygame
import pygame.draw
import numpy as np
from agent import *
from tools import *


SCREENSIZE = [1200, 400] # walls.csv
#SCREENSIZE = [1200, 650] # walls2.csv
RESOLUTION = 180
AGENTSNUM = 12
GROUPSNUM = 2
MAXGROUPSIZE = 6
MAXSUBGROUPSIZE = 3
BACKGROUNDCOLOR = [255, 255, 255]
LINECOLOR = [255,0,0]
AGENTSIZE = 9
AGENTTHICKNESS = 3
WALLSFILE = "walls.csv"

pygame.init()
screen = pygame.display.set_mode(SCREENSIZE)
pygame.display.set_caption('Social Force Model - Crosswalk')
clock = pygame.time.Clock()

# initialize walls
walls = []
for line in open(WALLSFILE, newline='', encoding="utf-8-sig"):
    coords = line.split(",")
    wall = []
    wall.append(float(coords[0]))
    wall.append(float(coords[1]))
    wall.append(float(coords[2]))
    wall.append(float(coords[3]))
    walls.append(wall)


# initialize agents
agents = []

for n in range(AGENTSNUM):
    group_id = (int)(n / MAXGROUPSIZE)
    subgroup_id = (int)((n % MAXGROUPSIZE) / MAXSUBGROUPSIZE)

    if n % MAXGROUPSIZE == 0:
        agents.append([])

    if n % MAXSUBGROUPSIZE == 0:
        agents[group_id].append([])

    agent = Agent(n, group_id, subgroup_id)
    agents[group_id][subgroup_id].append(agent)


running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.MOUSEBUTTONDOWN:
            (mouseX, mouseY) = pygame.mouse.get_pos()

    screen.fill(BACKGROUNDCOLOR)

    # draw walls
    for wall in walls:
        startPos = np.array([wall[0],wall[1]])
        endPos = np.array([wall[2],wall[3]])
        startPx = startPos*10 #worldCoord2ScreenCoord(startPos,SCREENSIZE,RESOLUTION)
        endPx = endPos*10 #worldCoord2ScreenCoord(endPos,SCREENSIZE,RESOLUTION)
        pygame.draw.line(screen, LINECOLOR, startPx.astype(int), endPx.astype(int))

    for group in agents:
        for subgroup in group:
            for agent in subgroup:
                agent.direction = normalize(agent.dest - agent.pos)
                agent.desiredV = agent.desiredSpeed * agent.direction

                adapt = agent.adaptVel()

                # initial forces values
                peopleInter = 0.0
                wallInter = 0.0
                groupVis = 0.0
                groupAtt = 0.0
                ownGroupRep = 0.0
                otherGroupRep = 0.0

                # wall interaction
                for wall in walls:
                    wallInter += agent.wallInteraction(wall)

                # people interaction
                for groupj in agents:
                    for subgroupj in groupj:
                        for agentj in subgroupj:

                            if agent.agentId != agentj.agentId:
                                peopleInter += agent.peopleInteraction(agentj)

                # list of group members excluding current ped
                agentGroup = []
                for sub in group:
                    for mem in sub:
                        if mem.agentId != agent.agentId:
                            agentGroup.append(mem)

                # group visual and attraction forces
                if len(agentGroup) > 0:
                    groupVis = agent.groupVisual(agentGroup)
                    groupAtt = agent.groupAttraction(agentGroup + [agent])

                # same group repulsion
                for agentj in agentGroup:
                    ownGroupRep += agent.ownGroupRepulsion(agentj)

                groupInter = groupVis + groupAtt + ownGroupRep

                # other groups repulsion
                for gid,g in enumerate(agents):
                    if gid != agent.groupId:
                        # create list of 'other group' members
                        otherGroup = []
                        for sub in g:
                            otherGroup += sub

                        otherGroupRep += agent.otherGroupRepulsion(otherGroup)

                #print(otherGroupRep)

                # subgroup forces
                subgroupForce = agent.subgroupForces(group)

                sumForce = adapt + wallInter + peopleInter + groupInter# + otherGroupRep + subgroupForce

                accl = sumForce / agent.mass

                agent.actualV = agent.actualV + accl*0.5 # consider dt = 0.5

                agent.pos = agent.pos + agent.actualV*0.5

                if (np.linalg.norm(agent.pos - agent.dest) < 2) & (agent.Goal == 0):
                    agent.Goal = 1
                    agent.timeOut = pygame.time.get_ticks()
                    #agent.timeOut = clock.get_time()/1000.0
                    print('Agent ', agent.agentId, 'reached goal at ', agent.timeOut)

    for group in agents:
        for subgroup in group:
            for agent in subgroup:
                scPos = (agent.pos*10).astype(int) #worldCoord2ScreenCoord(agent.pos, SCREENSIZE, RESOLUTION)
                endPos = ((agent.pos + agent.actualV) * 10).astype(int)
                endPosDV = ((agent.pos + agent.desiredV) * 10).astype(int)

                pygame.draw.circle(screen, agent.color, scPos, AGENTSIZE, AGENTTHICKNESS)
                pygame.draw.circle(screen, agent.subgroupColor, scPos, 5, 3)
                pygame.draw.line(screen, agent.color, scPos, endPos, 2)
                pygame.draw.line(screen, [255,60,0], scPos, endPosDV, 2)

    pygame.display.flip()
    clock.tick(20)
    #clock.get_time