"""Before/after figures for the diagnosed movement and hold failures."""
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from .plot_predictions import read_csv,traces,title
from .refit_motion import OUT


def main():
    font=Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    if font.exists():plt.rcParams['font.family']=font_manager.FontProperties(fname=str(font)).get_name()
    plt.rcParams['axes.unicode_minus']=False
    data=read_csv(OUT/'all_runs.csv');dest=OUT/'graphs';dest.mkdir(exist_ok=True)
    choices=[('20261006_134235_833624_S2','04 · S2',20,65),
             ('20261006_134834_301155_S1a','05 · S1a',40,115),
             ('20261006_135218_489972_S1b','06 · S1b',35,100),
             ('20261006_141418_972450_S6','10 · S6 seed 0',100,280),
             ('20261006_150506_052414_S6','13 · S6 seed 3 · 개발',200,620)]
    for zoom in (False,True):
        fig,axes=plt.subplots(len(choices),2,figsize=(18,18))
        for row,(name,label,a,b) in enumerate(choices):
            mask=data['run_id']==name
            if zoom:mask &= (data['t_mono_s']>=a)&(data['t_mono_s']<=b)
            after={k:v[mask] for k,v in data.items()};before=dict(after)
            before['angle_nominal_refit_deg']=after['angle_nominal_previous_deg']
            before['angle_residual_selected_deg']=after['angle_residual_previous_deg']
            curves=[]
            for col,(group,heading) in enumerate([(before,'이전 v3'),(after,'재피팅 v4')]):
                ax=axes[row,col];traces(ax,group,group['t_mono_s'])
                ax.set_title(title(group,f'{label} / {heading}'),fontsize=10)
                ax.set_xlabel('실행 시간 [s]')
                curves.extend([group[k] for k in ('angle_actual_deg','angle_nominal_refit_deg','angle_residual_selected_deg')])
            lo=min(np.min(x) for x in curves);hi=max(np.max(x) for x in curves)
            for ax in axes[row]:ax.set_ylim(lo-2,hi+2)
        handles,labels=axes[0,0].get_legend_handles_labels()
        fig.legend(handles,labels,loc='upper center',ncol=3)
        fig.suptitle('이동·유지 거동 재피팅 전후 — 같은 압력·같은 초기 조건, 좌우 같은 각도 축',y=.975)
        fig.tight_layout(rect=(0,0,1,.96))
        stem='16_before_after_zoom' if zoom else '15_before_after'
        fig.savefig(dest/(stem+'.png'),dpi=170)
        fig.savefig(dest/(stem+'.pdf'))
        plt.close(fig)
    print('Saved before/after overview and zoom plots',flush=True)


if __name__=='__main__':main()
