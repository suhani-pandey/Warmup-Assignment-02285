from abc import ABC, abstractmethod
import sys
from collections import deque

from searchclient.state import State


class Heuristic(ABC):
    DEBUG_HEURISTIC = False
    def __init__(self, initial_state: State) -> None: pass
    @abstractmethod
    def h(self, state: State) -> int: ...
    @abstractmethod
    def f(self, state: State) -> int: ...
    @abstractmethod
    def __repr__(self) -> str: ...


class HeuristicGoalCount(Heuristic):
    def __init__(self, initial_state: State) -> None:
        super().__init__(initial_state)
    def h(self, state: State) -> int:
        count = 0
        for row in range(len(State.goals)):
            for col in range(len(State.goals[row])):
                goal = State.goals[row][col]
                if "0" <= goal <= "9":
                    n = ord(goal) - ord("0")
                    if state.agent_rows[n] != row or state.agent_cols[n] != col:
                        count += 1
                elif "A" <= goal.upper() <= "Z":
                    if state.boxes[row][col].upper() != goal.upper():
                        count += 1
        return count


class HeuristicAdvanced(Heuristic):
    """
    h(s) = goal_count * GOAL_WEIGHT
         + dead_box_count * DEAD_BOX_WEIGHT
         + hungarian_box_goal_cost
         + min_agent_to_unsatisfied_box
         + agent_goal_cost
    """
    
    GOAL_WEIGHT = 10_000
    DEAD_BOX_WEIGHT = 20_000  # 2x goal weight per dead box
    
    def __init__(self, initial_state: State) -> None:
        super().__init__(initial_state)
        
        num_rows = len(State.walls)
        num_cols = len(State.walls[0]) if num_rows > 0 else 0
        
        # Collect goals
        self.goal_positions = set()
        self.agent_goals = []
        self.box_goals_by_type = {}
        
        for r in range(len(State.goals)):
            for c in range(len(State.goals[r])):
                g = State.goals[r][c]
                if g == "": continue
                if "0" <= g <= "9":
                    self.agent_goals.append((r, c, ord(g)-ord("0")))
                elif "A" <= g.upper() <= "Z":
                    gu = g.upper()
                    self.goal_positions.add((r, c))
                    if gu not in self.box_goals_by_type:
                        self.box_goals_by_type[gu] = []
                    self.box_goals_by_type[gu].append((r, c))
        
        # Dead cells: corners + edge propagation 
        self.is_corner = [[False]*num_cols for _ in range(num_rows)]
        self.is_dead_cell = [[False]*num_cols for _ in range(num_rows)]
        
        for r in range(num_rows):
            for c in range(num_cols):
                if State.walls[r][c]: continue
                wn = (r-1<0) or State.walls[r-1][c]
                ws = (r+1>=num_rows) or State.walls[r+1][c]
                we = (c+1>=num_cols) or State.walls[r][c+1]
                ww = (c-1<0) or State.walls[r][c-1]
                if (wn and we) or (wn and ww) or (ws and we) or (ws and ww):
                    self.is_corner[r][c] = True
        
        for r in range(num_rows):
            for c in range(num_cols):
                if self.is_corner[r][c] and (r,c) not in self.goal_positions:
                    self.is_dead_cell[r][c] = True
        
        self._detect_edges(num_rows, num_cols)
        
        # BFS distances
        self.goal_distances = {}
        for r in range(len(State.goals)):
            for c in range(len(State.goals[r])):
                g = State.goals[r][c]
                if g == "": continue
                self.goal_distances[(r,c)] = self._bfs(r, c)
        self.all_distances = {}
    
    def _detect_edges(self, R, C):
        for r in range(R):
            for c in range(C):
                if not self.is_dead_cell[r][c]: continue
                if r-1<0 or State.walls[r-1][c]:
                    self._mark(r,c,0,1, lambda rr,cc: rr-1<0 or State.walls[rr-1][cc], R,C)
                if r+1>=R or State.walls[r+1][c]:
                    self._mark(r,c,0,1, lambda rr,cc: rr+1>=R or State.walls[rr+1][cc], R,C)
                if c-1<0 or State.walls[r][c-1]:
                    self._mark(r,c,1,0, lambda rr,cc: cc-1<0 or State.walls[rr][cc-1], R,C)
                if c+1>=C or State.walls[r][c+1]:
                    self._mark(r,c,1,0, lambda rr,cc: cc+1>=C or State.walls[rr][cc+1], R,C)
    
    def _mark(self, sr, sc, dr, dc, wchk, R, C):
        seg = []; r,c = sr+dr, sc+dc; has_goal = False
        while 0<=r<R and 0<=c<C and not State.walls[r][c]:
            if not wchk(r,c): break
            if (r,c) in self.goal_positions: has_goal = True
            seg.append((r,c))
            if self.is_corner[r][c]:
                if self.is_dead_cell[r][c] and not has_goal:
                    for s,t in seg:
                        if (s,t) not in self.goal_positions:
                            self.is_dead_cell[s][t] = True
                break
            r+=dr; c+=dc
    
    def _bfs(self, sr, sc):
        dist = {}; q = deque([(sr,sc,0)]); vis = {(sr,sc)}
        while q:
            r,c,d = q.popleft(); dist[(r,c)] = d
            for dr,dc in [(-1,0),(1,0),(0,-1),(0,1)]:
                nr,nc = r+dr, c+dc
                if 0<=nr<len(State.walls) and 0<=nc<len(State.walls[0]):
                    if not State.walls[nr][nc] and (nr,nc) not in vis:
                        vis.add((nr,nc)); q.append((nr,nc,d+1))
        return dist

    def _gdist(self, gr, gc, r, c):
        dd = self.goal_distances.get((gr,gc),{})
        return dd.get((r,c), abs(gr-r)+abs(gc-c))

    def _cdist(self, r1,c1,r2,c2):
        for a,b,x,y in [(r1,c1,r2,c2),(r2,c2,r1,c1)]:
            if (a,b) in self.goal_distances:
                d = self.goal_distances[(a,b)].get((x,y))
                if d is not None: return d
        if (r1,c1) not in self.all_distances:
            self.all_distances[(r1,c1)] = self._bfs(r1,c1)
        d = self.all_distances[(r1,c1)].get((r2,c2))
        return d if d is not None else abs(r1-r2)+abs(c1-c2)

    def _hungarian(self, cm):
        n=len(cm)
        if n==0: return 0
        m=len(cm[0]) if n>0 else 0
        if m==0: return 0
        sz=max(n,m); mx=[[0]*sz for _ in range(sz)]
        for i in range(n):
            for j in range(m): mx[i][j]=cm[i][j]
        INF=float('inf')
        u=[0]*(sz+1);v=[0]*(sz+1);p=[0]*(sz+1);way=[0]*(sz+1)
        for i in range(1,sz+1):
            p[0]=i;j0=0;minv=[INF]*(sz+1);used=[False]*(sz+1)
            while True:
                used[j0]=True;i0=p[j0];delta=INF;j1=-1
                for j in range(1,sz+1):
                    if not used[j]:
                        cur=mx[i0-1][j-1]-u[i0]-v[j]
                        if cur<minv[j]:minv[j]=cur;way[j]=j0
                        if minv[j]<delta:delta=minv[j];j1=j
                for j in range(sz+1):
                    if used[j]:u[p[j]]+=delta;v[j]-=delta
                    else:minv[j]-=delta
                j0=j1
                if p[j0]==0:break
            while j0:p[j0]=p[way[j0]];j0=way[j0]
        t=0
        for j in range(1,sz+1):
            if p[j]!=0 and p[j]<=n and j<=m: t+=cm[p[j]-1][j-1]
        return t

    def h(self, state: State) -> int:
        # Count boxes on dead cells (for soft penalty)
        dead_box_count = 0
        for r in range(len(state.boxes)):
            for c in range(len(state.boxes[r])):
                if state.boxes[r][c] != '' and state.boxes[r][c] != ' ':
                    if self.is_dead_cell[r][c]:
                        dead_box_count += 1
        
        # Goal count (primary)
        uc = 0
        for row in range(len(State.goals)):
            for col in range(len(State.goals[row])):
                g = State.goals[row][col]
                if g == "": continue
                if "0"<=g<="9":
                    n=ord(g)-ord("0")
                    if state.agent_rows[n]!=row or state.agent_cols[n]!=col: uc+=1
                elif "A"<=g.upper()<="Z":
                    if state.boxes[row][col].upper()!=g.upper(): uc+=1
        
        total = uc * self.GOAL_WEIGHT
        
        # Soft dead-box penalty
        total += dead_box_count * self.DEAD_BOX_WEIGHT
        
        # Distance refinement
        mad = 0
        for gr,gc,an in self.agent_goals:
            d = self._gdist(gr,gc,state.agent_rows[an],state.agent_cols[an])
            if d>mad: mad=d
        total += mad
        
        aub = []
        for letter, goals in self.box_goals_by_type.items():
            boxes=[]
            for r in range(len(state.boxes)):
                for c in range(len(state.boxes[r])):
                    if state.boxes[r][c].upper()==letter: boxes.append((r,c))
            if not goals: continue
            if not boxes: total+=len(goals)*100; continue
            
            ugi=[]
            for gi,(gr,gc) in enumerate(goals):
                if state.boxes[gr][gc].upper()!=letter: ugi.append(gi)
            if not ugi: continue
            
            ub=[]
            for br,bc in boxes:
                if State.goals[br][bc].upper()!=letter: ub.append((br,bc))
            if not ub and ugi: ub=boxes[:]
            aub.extend(ub)
            
            ug=[goals[gi] for gi in ugi]
            if not ub: total+=len(ug)*100; continue
            cm=[[self._gdist(gr,gc,br,bc) for gr,gc in ug] for br,bc in ub]
            total += self._hungarian(cm)
        
        if aub and len(state.agent_rows)>0:
            ar,ac=state.agent_rows[0],state.agent_cols[0]
            md=min(self._cdist(ar,ac,br,bc) for br,bc in aub)
            if md<float('inf'): total+=md
        
        return total


class HeuristicAStar(HeuristicAdvanced):
    def __init__(self, initial_state: State) -> None: super().__init__(initial_state)
    def f(self, state: State) -> int: return state.g + self.h(state)
    def __repr__(self): return "A* evaluation"

class HeuristicAStarGoalCount(HeuristicGoalCount):
    def __init__(self, initial_state: State) -> None: super().__init__(initial_state)
    def f(self, state: State) -> int: return state.g + self.h(state)
    def __repr__(self): return "A* with Goal Count evaluation"

class HeuristicWeightedAStar(HeuristicAdvanced):
    def __init__(self, initial_state: State, w: int) -> None:
        super().__init__(initial_state); self.w = w
    def f(self, state: State) -> int: return state.g + self.w * self.h(state)
    def __repr__(self): return f"WA*({self.w}) evaluation"

class HeuristicGreedy(HeuristicAdvanced):
    def __init__(self, initial_state: State) -> None: super().__init__(initial_state)
    def f(self, state: State) -> int: return self.h(state)
    def __repr__(self): return "greedy evaluation"

class HeuristicGreedyGoalCount(HeuristicGoalCount):
    def __init__(self, initial_state: State) -> None: super().__init__(initial_state)
    def f(self, state: State) -> int: return self.h(state)
    def __repr__(self): return "greedy with Goal Count evaluation"
