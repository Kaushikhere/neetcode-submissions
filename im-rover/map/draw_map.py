# Draws the practice field in cm from the same numbers as firmware/im_rover/field_map.h
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt, math
W,L=122,244
walls=[(19.5,198.6,46.6,140.2),(56.8,219.3,83.7,156.3),(16.2,173.6,68.0,192.6),(42.2,155.9,79.0,175.4),(86.1,227.8,103.6,181.3),(97.5,209.3,108.3,218.1)]
line=[(91.9,114.7),(78.8,119.9),(63.9,123.5),(48.7,124.8),(33.6,123.6),(22.6,120.2),(16.7,114.4),(15.5,106.6),(17.6,93.9),(21.4,84.0),(26.6,76.4),(34.8,69.2),(44.9,64.8),(56.2,63.1),(67.3,63.6),(77.0,66.5),(84.6,72.7),(91.1,83.8),(96.5,97.8),(100.3,110.6),(103.1,124.4),(103.9,139.3),(102.2,152.6),(96.9,163.9),(89.4,177.4),(81.4,195.1),(73.8,210.7),(68.1,221.7),(64.6,225.5),(51.6,221.9),(35.4,214.8),(17.2,209.7),(31.5,204.2),(47.7,198.9)]
decoys=[[(13.1,158.9),(22.6,146.5),(32.0,130.9)],[(61.7,155.7),(65.8,140.9),(69.2,127.0)],[(70.9,210.7),(77.6,223.4),(87.3,228.9),(97.0,230.7),(103.7,224.9),(104.4,219.3)]]
marks=[(13.7,45.3,-50),(46.1,34.0,40),(56.3,19.2,88),(75.7,48.9,-123),(85.4,34.1,-2),(105.6,16.2,136)]
T1=(57.1,96.9); T2=(49.0,188.5)
fig,ax=plt.subplots(figsize=(7.2,13.2),dpi=150)
ax.add_patch(plt.Rectangle((0,0),W,L,fc="#EDECE7",ec="none"))
for r in [(0,0,W,5),(0,L-5,W,5),(0,0,5,L),(W-5,0,5,L)]: ax.add_patch(plt.Rectangle(r[:2],r[2],r[3],fc="#161616",ec="none"))
ax.plot([0,W],[0,0],c="#2D8A55",lw=6,solid_capstyle="butt"); ax.plot([0,0],[0,60],c="#2D8A55",lw=6,solid_capstyle="butt"); ax.plot([W,W],[0,60],c="#2D8A55",lw=6,solid_capstyle="butt")
ax.text(61,-6,"BACK WALL 30 cm (122)",ha="center",fontsize=8,color="#2D8A55"); ax.text(-3,30,"30 cm side wall, 60 long",rotation=90,va="center",ha="right",fontsize=7,color="#2D8A55"); ax.text(W+3,30,"30 cm side wall, 60 long",rotation=90,va="center",fontsize=7,color="#2D8A55")
for x1,y1,x2,y2 in walls: ax.plot([x1,x2],[y1,y2],c="#3F7A58",lw=4,solid_capstyle="butt")
xs,ys=zip(*line); ax.plot(xs,ys,c="k",lw=3.2,solid_joinstyle="round")
for d in decoys: xs,ys=zip(*d); ax.plot(xs,ys,c="k",lw=3.2,alpha=.85)
ax.text(14,150,"decoy",fontsize=7,rotation=-55); ax.text(66,138,"decoy",fontsize=7,rotation=-75); ax.text(92,233,"branch (decoy)",fontsize=7)
for i,(x,y,h) in enumerate(marks,1):
    c,s=math.cos(math.radians(h)),math.sin(math.radians(h))
    ax.plot([x,x+15*c],[y,y+15*s],c="#E0A526",lw=1.4); ax.plot([x+7.5*s,x-7.5*s],[y-7.5*c,y+7.5*c],c="#E0A526",lw=1.4)
    ax.plot(x,y,"o",c="#2B3A8F",ms=3); ax.text(x+1.5,y-5,str(i),color="#2B3A8F",fontsize=9,weight="bold")
ax.add_patch(plt.Circle(T1,3,fc="none",ec="#D41E78",lw=2)); ax.plot(*T1,"x",c="#D41E78"); ax.text(T1[0]+4,T1[1]+3,f"T1 ({T1[0]}, {T1[1]})",color="#D41E78",fontsize=9,weight="bold")
ax.add_patch(plt.Circle(T2,2.5,fc="#6A3CC6",ec="none")); ax.text(T2[0]-40,T2[1]-9,f"T2 ≈ ({T2[0]}, {T2[1]})",color="#6A3CC6",fontsize=8,weight="bold")
# our plan: home spot below T1, the shot, pickup, route
blue="#2357C4"
ax.plot([T1[0],T1[0]],[4.5,T1[1]],c=blue,lw=1.6,ls="--"); ax.plot(T1[0],4.5,"D",c=blue,ms=5); ax.text(T1[0]+2,8,"home spot: squared on the\nback wall, straight below T1",color=blue,fontsize=7)
ax.annotate("",xy=(96.5,97),xytext=(T1[0],97),arrowprops=dict(arrowstyle="->",color=blue,lw=1.6)); ax.text(70,91,"pick up line (14–32 cm)",color=blue,fontsize=7)
for rt,txt,(x,y) in [(17,"ignore LEFT: oval end",(76,112)),(62,"SLOW: passage",(104,160)),(120,"ignore RIGHT: branch",(79,212)),(188,"hairpin: scripted\n124° left pivot",(2,214)),(220,"line end → face rung,\nstop at taught FRONT",(52,199))]:
    ax.text(x,y,f"{txt}",fontsize=6.5,color=blue,ha="left",bbox=dict(fc="white",ec=blue,lw=.5,alpha=.85,pad=1.5))
ax.set_xlim(-12,134); ax.set_ylim(-12,250); ax.set_aspect("equal")
ax.set_xticks(range(0,123,10)); ax.set_yticks(range(0,245,10)); ax.grid(True,lw=.3,alpha=.5); ax.tick_params(labelsize=6)
ax.set_xlabel("x: cm from the LEFT wall's inner face",fontsize=8); ax.set_ylabel("y: cm from the BACK wall's inner face",fontsize=8)
ax.set_title("IM practice field (traced from photos, ±2–4 cm)\nyellow T = start marks (dot = axle, long leg = facing)",fontsize=9)
plt.tight_layout(); plt.savefig("practice_field_map.png")
