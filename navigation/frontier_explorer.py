"""Conservative frontier selection on known free space; no ROS or actuators."""
import math
from collections import deque


def select_frontiers(grid, pose, clearance_m, excluded=()):
    w,h,res=grid['width'],grid['height'],grid['resolution']
    data=grid['data'];ox,oy,theta=grid['origin']
    if w<1 or h<1 or w*h>1_000_000 or len(data)!=w*h or not math.isfinite(res) or res<=0:
        raise ValueError('地图尺寸或分辨率不适合自动探索')
    c,s=math.cos(theta),math.sin(theta)
    def cell(x,y):
        dx,dy=x-ox,y-oy
        return math.floor((c*dx+s*dy)/res),math.floor((-s*dx+c*dy)/res)
    def world(i):
        x,y=(i%w+.5)*res,(i//w+.5)*res
        return ox+c*x-s*y,oy+s*x+c*y
    def neighbors(i, diagonal=False):
        x,y=i%w,i//w
        steps=((-1,0),(1,0),(0,-1),(0,1))
        if diagonal:steps+=((-1,-1),(-1,1),(1,-1),(1,1))
        for dx,dy in steps:
            if 0<=x+dx<w and 0<=y+dy<h:yield (y+dy)*w+x+dx
    # Chebyshev distance inflates ALL nonfree cells, including unknown and map edges.
    # A square of this radius encloses the chassis in every heading.
    radius=math.ceil(clearance_m/res)
    distance=[radius+1]*(w*h);queue=deque()
    for i,v in enumerate(data):
        if not 0<=v<25 or i%w in (0,w-1) or i//w in (0,h-1):distance[i]=0;queue.append(i)
    while queue:
        i=queue.popleft()
        if distance[i]>=radius:continue
        for j in neighbors(i,True):
            if distance[j]>distance[i]+1:distance[j]=distance[i]+1;queue.append(j)
    x,y=cell(pose['x'],pose['y'])
    if not 0<=x<w or not 0<=y<h or distance[y*w+x]<=radius:
        raise ValueError('机器人起点没有足够的已知自由空间；请在开阔地面检查地图和定位')
    start=y*w+x;travel={start:0};queue=deque([start])
    while queue:
        i=queue.popleft()
        for j in neighbors(i):
            if distance[j]>radius and j not in travel:travel[j]=travel[i]+1;queue.append(j)
    frontiers={i for i,v in enumerate(data) if 0<=v<25 and any(data[j]<0 for j in neighbors(i))}
    groups=[]
    while frontiers:
        seed=frontiers.pop();group=[seed];queue=deque([seed])
        while queue:
            for j in neighbors(queue.popleft(),True):
                if j in frontiers:frontiers.remove(j);group.append(j);queue.append(j)
        if len(group)*res>=.25:groups.append(group)
    candidates=[]
    for group in groups:
        # Search behind each frontier for a reachable standoff pose, never in unknown space.
        queue=deque((i,0) for i in group);seen=set(group);options=[]
        while queue:
            i,depth=queue.popleft()
            if i in travel:options.append(i);continue
            if depth>radius+4:continue
            for j in neighbors(i):
                if j not in seen and 0<=data[j]<25:seen.add(j);queue.append((j,depth+1))
        options=sorted(options,key=lambda i:travel[i])
        for i in options:
            px,py=world(i)
            if math.hypot(px-pose['x'],py-pose['y'])<.25:continue
            if any(math.hypot(px-e['x'],py-e['y'])<.65 for e in excluded):continue
            fx=sum(world(j)[0] for j in group)/len(group);fy=sum(world(j)[1] for j in group)/len(group)
            candidates.append(dict(x=px,y=py,yaw=math.atan2(fy-py,fx-px),gain_m=len(group)*res,
                                   distance_m=travel[i]*res,score=travel[i]*res-.3*min(3,len(group)*res)))
            break
    candidates.sort(key=lambda p:p['score'])
    return dict(candidates=candidates[:20],frontier_groups=len(groups),reachable_cells=len(travel),
                complete=not groups,clearance_m=clearance_m)
