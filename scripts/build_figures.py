"""Build vector figures from archived numeric curves and reported tables.

No new simulation or image digitization is performed. FIG curve extraction uses
SciPy; normal figure regeneration uses versioned CSVs and NumPy/Matplotlib.
"""
from pathlib import Path
import argparse
import csv
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / 'data/figure-traces'
FIG = ROOT / 'docs/report/figures'


def export_curves():
    from scipy.io import loadmat
    m = loadmat(ROOT / 'archive/figures/plot.fig', simplify_cells=True)
    lines = m['hgS_070000']['children']['children'] if isinstance(m['hgS_070000']['children'], dict) else m['hgS_070000']['children'][0]['children']
    provenance = {}
    for item in lines:
        if item['type'] != 'graph2d.lineseries':
            continue
        props = item['properties']
        label = str(props.get('DisplayName', '')).lower()
        if label not in ['reference', 'measured']:
            continue
        xyz = np.column_stack([props[k] for k in ['XData','YData','ZData']])
        np.savetxt(DATA / f'task_path_{label}.csv', xyz, delimiter=',',
                   header='x_m,y_m,z_m', comments='', fmt='%.17g')
        provenance[label] = {'source':'archive/figures/plot.fig', 'display_name':label,
                             'samples':len(xyz), 'has_time_vector':False,
                             'method':'direct MATLAB FIG numeric XData/YData/ZData extraction'}
    (DATA / 'path-provenance.json').write_text(json.dumps(provenance, indent=2)+'\n')


def save(fig, name):
    for ext in ['pdf', 'svg', 'png']:
        fig.savefig(FIG / f'{name}.{ext}', dpi=300 if ext == 'png' else None,
                    bbox_inches='tight', pad_inches=.04)
    plt.close(fig)


def main():
    plt.rcParams.update({'font.family':'serif','font.serif':['DejaVu Serif'],
                         'font.size':8,'axes.labelsize':8,'legend.fontsize':7,
                         'xtick.labelsize':7,'ytick.labelsize':7,
                         'axes.linewidth':.6,'lines.linewidth':1.0,
                         'pdf.fonttype':42,'ps.fonttype':42})
    a = np.genfromtxt(DATA/'bulk_modulus_summary.csv',delimiter=',',names=True)
    fig, ax = plt.subplots(1, 2, figsize=(7.16,2.65),layout='constrained')
    for field,label,style,color in [('j1_delay_ms','Joint 1','o-','#155a87'),
                                     ('j3_delay_ms','Joint 3','s--','#a54e00'),
                                     ('j4_delay_ms','Joint 4','^:','#466b34')]:
        ax[0].plot(a['bulk_modulus_gpa'],a[field],style,color=color,label=label,ms=4)
    ax[0].set(xlabel=r'Bulk modulus $\beta$ (GPa)',ylabel='Valve-to-joint 50% delay (ms)',ylim=(0,950))
    ax[0].legend(frameon=False,loc='upper right')
    ax[1].plot(a['bulk_modulus_gpa'],a['ee_rms_mm'],'o-',color='#155a87',label='RMS error',ms=4)
    ax[1].plot(a['bulk_modulus_gpa'],a['ee_mean_mm'],'s--',color='#a54e00',label='Mean error',ms=4)
    ax[1].set(xlabel=r'Bulk modulus $\beta$ (GPa)',ylabel='End-effector error (mm)',ylim=(0,950))
    ax[1].legend(frameon=False,loc='center right')
    for i,x in enumerate(ax):
        x.grid(True,color='.9',lw=.5);x.set_xticks([.5,1,1.5,2]);x.spines[['top','right']].set_visible(False)
        x.text(.02,.97,f'({chr(97+i)})',transform=x.transAxes,va='top')
    save(fig,'bulk-modulus-sensitivity')
    ref=np.loadtxt(DATA/'task_path_reference.csv',delimiter=',',skiprows=1)
    meas=np.loadtxt(DATA/'task_path_measured.csv',delimiter=',',skiprows=1)
    fig,axes=plt.subplots(1,2,figsize=(7.16,2.8),layout='constrained')
    for ax,channel,label in zip(axes,[1,2],['$y$ (m)','$z$ (m)']):
        ax.plot(ref[:,0],ref[:,channel],color='#155a87',label='Reference',lw=1.1)
        ax.plot(meas[:,0],meas[:,channel],color='#a54e00',ls='--',label='Simulated response',lw=.85)
        ax.set(xlabel='$x$ (m)',ylabel=label)
        ax.grid(True,color='.9',lw=.5);ax.spines[['top','right']].set_visible(False)
        ax.locator_params(axis='y',nbins=4)
    axes[0].set_aspect('equal',adjustable='datalim')
    axes[0].legend(frameon=False,loc='upper right')
    save(fig,'task-space-path')
    fig,ax=plt.subplots(figsize=(7.16,2.0));ax.set(xlim=(0,10),ylim=(0,3));ax.axis('off')
    boxes=[(.1,1.3,1.8,.8,'Joint reference\n& controller'),(2.7,1.3,1.8,.8,'Valve / fluid\ndynamics'),(5.3,1.3,1.8,.8,'Cylinder force\n& geometry'),(7.9,1.3,1.9,.8,'Rigid multibody\nmechanics')]
    for x,y,w,h,label in boxes:
        ax.add_patch(FancyBboxPatch((x,y),w,h,boxstyle='round,pad=.03',fc='#f0f5f8',ec='#526c7b',lw=.8))
        ax.text(x+w/2,y+h/2,label,ha='center',va='center',fontsize=8)
    for start,end in [(1.95,2.65),(4.55,5.25),(7.15,7.85)]:
        ax.add_patch(FancyArrowPatch((start,1.7),(end,1.7),arrowstyle='-|>',mutation_scale=10,color='#155a87',lw=1))
    ax.annotate('',xy=(3.6,1.25),xytext=(8.8,.6),arrowprops=dict(arrowstyle='->',connectionstyle='angle,angleA=180,angleB=90,rad=8',color='#a54e00',lw=.9))
    ax.text(6.7,.25,'Load / piston-motion feedback',ha='center',fontsize=7,color='#a54e00')
    ax.annotate('',xy=(1,1.25),xytext=(8.8,.95),arrowprops=dict(arrowstyle='->',connectionstyle='angle,angleA=180,angleB=90,rad=8',color='#526c7b',lw=.7))
    ax.text(6.3,1.02,'Joint position / velocity feedback',ha='center',fontsize=7)
    save(fig,'coupled-architecture')
    print('Built sensitivity, path, and architecture figures as PDF/SVG/300 dpi PNG.')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--extract-fig',action='store_true')
    if p.parse_args().extract_fig:export_curves()
    main()
