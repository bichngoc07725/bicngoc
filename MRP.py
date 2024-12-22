from ortools.linear_solver import pywraplp
n,m,s,L=map(int,input().split())
t={}
c={}
edges=[]
A=[[] for i in range(n+1)]

for j in range(m):
    u,v,time,cost=map(int,input().split())
    t[(u,v)]=time
    c[(u,v)]=cost
    edges.append((u,v))
    A[v].append(u)

def MRP(n,t,c,s,L,edges,A):
    solver=pywraplp.Solver.CreateSolver("SCIP")
    max_time=sum(t[(u,v)] for u,v in edges)
    time={}
    X={}
    for i in range(1,n+1):
        time[i]= solver.NumVar(0,max_time,f"time_{i}")
    for u,v in edges:
        X[(u,v)]= solver.BoolVar(f"edge{u}_{v}")
    solver.Add(time[s]==0)
    for u,v in edges:
        solver.Add(time[u]+t[u,v]+max_time*(1-X[(u,v)])>=time[v])
        solver.Add(time[u]+t[u,v]+m*(X[(u,v)]-1)<=time[v])
    for j in range(1,n+1):
        if j !=s:
            solver.Add(time[i]<=L)
    for j in range(1, n+1):
        if j != s:
            solver.Add(sum(X[(i, j)] for i in A[j] ) == 1)
    solver.Minimize(solver.Sum(c[(u, v)] * X[(u, v)] for u, v in edges))
    status=solver.Solve()
    if status==pywraplp.Solver.OPTIMAL:
        print("Giá trị tối ưu của hàm mục tiêu (chi phí):", solver.Objective().Value())
    else:
        print("None")
MRP(n,t,c,s,L,edges,A)