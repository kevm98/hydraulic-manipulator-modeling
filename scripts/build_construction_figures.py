"""Rebuild technical construction diagrams. These are schematics, not run data."""
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, FancyArrowPatch

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'docs/report/figures'
INK, FLUID, SIGNAL, MECH = '#263743', '#245B83', '#A14B32', '#39715A'


def canvas(height):
    fig, ax = plt.subplots(figsize=(7.16, height))
    ax.set(xlim=(0, 14), ylim=(0, height*2))
    ax.set_aspect('equal'); ax.axis('off')
    return fig, ax


def box(ax, x, y, w, h, label, color=INK, fill='white', fs=8, dashed=False):
    ax.add_patch(Rectangle((x,y),w,h,edgecolor=color,facecolor=fill,
                           lw=.85,linestyle='--' if dashed else '-'))
    ax.text(x+w/2,y+h/2,label,ha='center',va='center',fontsize=fs,color=INK)


def line(ax, points, color=INK, dashed=False, arrow=False, lw=.9):
    ax.plot(*zip(*points),color=color,lw=lw,ls='--' if dashed else '-')
    if arrow:
        ax.add_patch(FancyArrowPatch(points[-2],points[-1],arrowstyle='-|>',
                                    mutation_scale=8,color=color,lw=lw))


def text(ax,x,y,s,fs=8,color=INK,ha='left',va='center'):
    ax.text(x,y,s,fontsize=fs,color=color,ha=ha,va=va)


def save(fig,name):
    for ext in ['pdf','svg','png']:
        fig.savefig(OUT/f'{name}.{ext}',dpi=300,bbox_inches='tight',pad_inches=.06)
    svg=OUT/f'{name}.svg'
    svg.write_text('\n'.join(line.rstrip() for line in svg.read_text().splitlines())+'\n')
    plt.close(fig)


def cylinder():
    fig,ax=canvas(4.5)
    text(ax,.2,8.65,'(a) Chamber geometry and fluid ports',9)
    box(ax,1.1,5.8,9.5,1.9,'')
    box(ax,1.15,5.85,4.3,1.8,'Chamber A\n$p_A, V_A(s), A_A$',FLUID,'#EFF5F9')
    box(ax,5.7,5.85,4.85,1.8,'',FLUID,'#F8F3EE')
    text(ax,8.3,7.25,'Chamber B: $p_B, V_B(s), A_B$',7,ha='center')
    box(ax,5.45,5.8,.25,1.9,'',INK,'#8E9BA5')
    box(ax,5.7,6.6,6.5,.3,'',INK,'#D6DFE5')
    line(ax,[(2.2,7.7),(2.2,8.1)],FLUID)
    line(ax,[(9.3,7.7),(9.3,8.1)],FLUID)
    text(ax,2.2,8.3,'A: $Q_A$ into chamber',7,FLUID,ha='center')
    text(ax,9.3,8.3,'B: $Q_B$ out of chamber',7,FLUID,ha='center')
    line(ax,[(5.55,5.5),(9.1,5.5)],INK,arrow=True)
    text(ax,7.3,5.2,r'Positive extension $s$, velocity $v=\dot{s}$',8,ha='center')
    text(ax,1.1,5.2,'C: casing',8,MECH)
    text(ax,11.5,6.1,'R: rod',8,MECH)
    text(ax,.2,4.55,'(b) CAD bodies, alignment joints, and power interface',9)
    xs=[.1,1.9,3.6,5.4,7.3,9.1,11.2]
    labels=['Link $k$','Spherical\njoint','Barrel\nCAD body','Prismatic\njoint','Rod\nCAD body','Cylindrical\njoint','Link $k+1$']
    for x,label in zip(xs,labels):box(ax,x,3.0,1.5,.85,label,fs=7)
    for a,b in zip(xs[:-1],xs[1:]):line(ax,[(a+1.5,3.425),(b,3.425)],MECH,lw=1.3)
    box(ax,.5,.1,3.4,1.45,'Double-acting actuator\nA/B: liquid ports\nC/R: mechanical ports\np: displacement input',FLUID,fs=6.6)
    box(ax,5.4,.1,2.8,1.45,'Translational\nMultibody Interface\nC/R: mechanical\nv: velocity input\nf: force output',MECH,fs=6.2)
    box(ax,10,.1,2.7,1.45,'External friction\nparallel across C/R',MECH,fs=6.5)
    line(ax,[(3.9,.8),(5.4,.8)],MECH,lw=1.3)
    line(ax,[(8.2,.8),(10,.8)],MECH,lw=1.3)
    line(ax,[(6.1,3),(6.1,2.45),(6.1,1.55)],SIGNAL,dashed=True,arrow=True)
    text(ax,6.3,2.35,'joint velocity v',7,SIGNAL)
    line(ax,[(5.65,1.55),(5.65,2.7),(5.65,3)],SIGNAL,dashed=True,arrow=True)
    text(ax,5.5,2.5,'force f',6.5,SIGNAL,ha='right')
    line(ax,[(5.4,3),(5.1,2.8),(4.4,2.8),(4.4,2),(2.3,2),(2.3,1.55)],SIGNAL,dashed=True,arrow=True)
    text(ax,2.3,2.25,'joint position s to actuator p',7,SIGNAL,ha='center')
    save(fig,'cylinder-construction')


def network():
    fig,ax=canvas(4.9)
    text(ax,.15,9.4,'Saved model network: four parallel hydraulic branches and one torque branch',9)
    box(ax,.2,8.1,3,.75,'Joint reference\n+ controller',fs=7)
    line(ax,[(3.2,8.475),(11.1,8.475)],SIGNAL)
    branch_x=[3.8,6.0,8.2,10.4]
    for i,x in enumerate(branch_x):
        text(ax,x+.95,7.7,f'$u_{i+1}$',7,SIGNAL)
        box(ax,x,5.55,1.6,1.75,'4/3 valve\n+ cylinder\n+ interface',FLUID,fs=7)
        text(ax,x+.4,5.34,'2',6,FLUID,ha='center')
        text(ax,x+1.2,5.34,'3',6,FLUID,ha='center')
        line(ax,[(x+.8,8.475),(x+.8,7.3)],SIGNAL,dashed=True,arrow=True)
        line(ax,[(x+.4,5.55),(x+.4,4.7)],FLUID)
        line(ax,[(x+1.2,5.55),(x+1.2,4.0)],FLUID)
        box(ax,x,6.85,1.6,.45,'',FLUID,fill='#EDF3F7')
        text(ax,x+.8,7.075,f'Actuator {i+1}',6.5,ha='center')
        box(ax,x,2.55,1.6,.7,f'Joint {i+1}\nCAD linkage',MECH,fs=7)
        line(ax,[(x+1.6,6.1),(x+1.9,6.1),(x+1.9,2.9),(x+1.6,2.9)],MECH,dashed=True)
    line(ax,[(1.35,4.7),(12.2,4.7)],FLUID,lw=1.3)
    line(ax,[(2.9,4.0),(12.2,4.0)],FLUID,lw=1.3)
    for x in branch_x:
        ax.add_patch(Circle((x+.4,4.7),.045,color=FLUID))
        ax.add_patch(Circle((x+1.2,4.0),.045,color=FLUID))
    ax.add_patch(Circle((3.5,4.7),.045,color=FLUID))
    ax.add_patch(Circle((12.2,4.0),.045,color=FLUID))
    text(ax,13.0,4.7,'N1',8,FLUID)
    text(ax,13.0,4.0,'N2',8,FLUID)
    box(ax,.2,5.35,2.25,.75,'Reservoir\n(atmospheric)',FLUID,fs=7)
    line(ax,[(1.35,5.35),(1.35,4.7)],FLUID)
    box(ax,.2,2.7,2.25,.85,'Fixed-displacement\npump',FLUID,fs=6.2)
    text(ax,.9,3.75,'B',7,FLUID)
    text(ax,2.58,3.15,'A',7,FLUID)
    line(ax,[(1.35,3.55),(1.35,4.7)],FLUID)
    line(ax,[(2.45,3.15),(2.9,3.15),(2.9,4.0)],FLUID)
    box(ax,.2,1.45,2.25,.65,'Speed source\n188 (stored)',fs=6.8)
    line(ax,[(1.35,2.1),(1.35,2.7)],MECH)
    box(ax,4.15,.9,2.15,.8,'Pressure relief\nA: N1; B: N2',FLUID,fs=6.5)
    line(ax,[(4.45,1.7),(3.5,1.7),(3.5,4.7)],FLUID)
    line(ax,[(6.0,1.7),(6.0,2.05),(12.2,2.05),(12.2,4.0)],FLUID)
    box(ax,7.0,.9,2.3,.8,'Fluid properties\nattached to N2',FLUID,fs=6.5)
    line(ax,[(9.3,1.3),(12.6,1.3),(12.6,4.0),(12.2,4.0)],FLUID)
    box(ax,10.1,.4,3.3,.8,'Accumulator / old relief\ncommented out',fs=6,dashed=True)
    box(ax,11.7,8.1,2.0,.75,'$u_5$: joint torque\n(no liquid branch)',MECH,fs=6.2)
    line(ax,[(11.1,8.475),(11.7,8.475)],SIGNAL,dashed=True,arrow=True)
    text(ax,.2,.5,'Blue: liquid   Green: mechanics   Dashed red: commands',6.0)
    text(ax,.2,.13,'2 / 3: serialized RConn indices at the actuator subsystem boundary.',6.0)
    save(fig,'hydraulic-network')


def valve():
    fig,ax=canvas(2.55)
    for i,(label,mode) in enumerate([('Positive spool','extend'),('Neutral spool','hold'),('Negative spool','retract')]):
        x=.3+i*4.6
        text(ax,x+1.8,4.7,f'({chr(97+i)}) {label}',9,ha='center')
        box(ax,x,1.4,3.6,2.7,'',FLUID)
        for px,pn in [(x+.65,'A'),(x+2.95,'B')]:text(ax,px,4.35,pn,8,FLUID,ha='center')
        for px,pn in [(x+.65,'P'),(x+2.95,'T')]:text(ax,px,1.1,pn,8,FLUID,ha='center')
        if mode=='extend':
            line(ax,[(x+.65,1.65),(x+.65,3.85)],FLUID,arrow=True)
            line(ax,[(x+2.95,3.85),(x+2.95,1.65)],FLUID,arrow=True)
        elif mode=='retract':
            line(ax,[(x+.65,1.65),(x+2.95,3.85)],FLUID,arrow=True)
            line(ax,[(x+.65,3.85),(x+2.95,1.65)],FLUID,arrow=True)
        else:
            for px in [x+.65,x+2.95]:
                line(ax,[(px,1.65),(px,2.15)],FLUID)
                line(ax,[(px-.3,2.15),(px+.3,2.15)],FLUID)
                line(ax,[(px,3.85),(px,3.35)],FLUID)
                line(ax,[(px-.3,3.35),(px+.3,3.35)],FLUID)
        text(ax,x+1.8,.55,{'extend':'P-A and B-T','retract':'P-B and A-T','hold':'All nominal paths closed'}[mode],8,ha='center')
    save(fig,'valve-routing')


if __name__=='__main__':
    plt.rcParams.update({'font.family':'serif','font.serif':['DejaVu Serif'],
                         'pdf.fonttype':42,'ps.fonttype':42})
    OUT.mkdir(parents=True,exist_ok=True)
    cylinder(); network(); valve()
    print('Built cylinder, complete network, and valve-routing diagrams.')
