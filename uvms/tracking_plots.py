"""Standard standalone plots and a native live telemetry panel."""
from pathlib import Path
import os
_cache = Path(__file__).resolve().parents[1]/"outputs/.matplotlib"
_cache.mkdir(parents=True,exist_ok=True)
os.environ.setdefault("MPLCONFIGDIR",str(_cache))


def make_live_window(parent=None):
    """Create a Qt/Matplotlib monitor without importing Qt for headless runs."""
    import numpy as np
    from PySide6.QtWidgets import QMainWindow,QTabWidget,QWidget,QVBoxLayout,QLabel
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg
    class Monitor(QMainWindow):
        def __init__(self):
            super().__init__(parent)
            self.setWindowTitle("轨迹跟踪 · 期望 / 实际 / 关节力矩")
            self.resize(1000,800)
            root = QWidget()
            self.setCentralWidget(root)
            layout = QVBoxLayout(root)
            self.label = QLabel("等待物理仿真数据；虚线：期望，实线：实际")
            layout.addWidget(self.label)
            self.tabs = QTabWidget()
            layout.addWidget(self.tabs)
            self.plots = []
            for title,desired,actual,labels,unit in (
                    ("关节角度","q_des_rad","q_act_rad","EDCB","rad"),
                    ("末端位置","position_des_m","position_act_m","XYZ","m"),
                    ("关节力矩","tau_requested_nm","tau_actual_nm","EDCB","N m"),
                    ("末端姿态","quaternion_des_wxyz","quaternion_act_wxyz","wxyz","quaternion")):
                figure = Figure(figsize=(9,7),layout="constrained")
                canvas = FigureCanvasQTAgg(figure)
                axes = figure.subplots(len(labels),1,sharex=True)
                lines = []
                for axis,label in zip(axes,labels):
                    d, = axis.plot([],[],"--",label="desired / requested")
                    a, = axis.plot([],[],label="actual")
                    axis.set_ylabel(f"{label} [{unit}]")
                    axis.grid(alpha=.3)
                    lines.append((d,a))
                axes[0].legend(fontsize=8)
                axes[-1].set_xlabel("Simulation time [s]")
                self.tabs.addTab(canvas,title)
                self.plots.append((canvas,axes,lines,desired,actual))

        def refresh(self,session):
            if not session.rows:
                return
            rows = session.rows[::max(1,len(session.rows)//1500)]
            t = np.array([r["time_s"] for r in rows])
            last = session.last
            self.label.setText(f"{session.status} · t={session.time:.3f}s · 位置误差 {last['position_error_m']*1000:.3f} mm · "
                               f"姿态误差 {np.rad2deg(last['orientation_error_rad']):.3f}° · 力矩限幅 {session.saturation_steps} 次")
            for canvas,axes,lines,desired,actual in self.plots:
                d = np.array([r[desired] for r in rows])
                a = np.array([r[actual] for r in rows])
                if actual.startswith("quaternion"):
                    a *= np.where(np.sum(d*a,axis=1)<0,-1,1)[:,None]
                for i,(axis,(dl,al)) in enumerate(zip(axes,lines)):
                    dl.set_data(t,d[:,i])
                    al.set_data(t,a[:,i])
                    axis.relim()
                    axis.autoscale_view()
                canvas.draw_idle()
    return Monitor()


def save_plots(data,directory):
    import matplotlib
    matplotlib.use("Agg")
    from matplotlib import pyplot as plt
    directory = Path(directory)
    for name,key,labels,unit in (("joint_tracking","q","EDCB","rad"),
                                 ("tcp_position","position","XYZ","m"),
                                 ("tcp_orientation","quaternion","wxyz","unit quaternion")):
        fig,axes = plt.subplots(len(labels),1,figsize=(10,2.2*len(labels)),sharex=True,layout="constrained")
        suffix = "_rad" if key == "q" else "_m" if key == "position" else "_wxyz"
        for i,(axis,label) in enumerate(zip(axes,labels)):
            axis.plot(data["time_s"],data[f"{key}_des{suffix}"][:,i],"--",label="desired")
            values = data[f"{key}_act{suffix}"].copy()
            if key == "quaternion":
                import numpy as np
                signs = np.where(np.sum(values*data[f"{key}_des{suffix}"],axis=1)<0,-1,1)
                values *= signs[:,None]
            axis.plot(data["time_s"],values[:,i],label="actual")
            axis.set_ylabel(f"{label} [{unit}]")
            axis.grid(alpha=.3)
        axes[0].legend()
        axes[-1].set_xlabel("Simulation time [s]")
        fig.savefig(directory/f"{name}.png",dpi=140)
        plt.close(fig)
    fig,axes = plt.subplots(4,1,figsize=(10,9),sharex=True,layout="constrained")
    for i,axis in enumerate(axes):
        axis.plot(data["time_s"],data["tau_requested_nm"][:,i],"--",label="requested")
        axis.plot(data["time_s"],data["tau_actual_nm"][:,i],label="actual actuator")
        axis.set_ylabel(f"{'EDCB'[i]} [N m]")
        axis.grid(alpha=.3)
    axes[0].legend()
    axes[-1].set_xlabel("Simulation time [s]")
    fig.savefig(directory/"joint_torque.png",dpi=140)
    plt.close(fig)
    if "grasp_contact_sides" in data:
        import numpy as np
        fig,axes = plt.subplots(3,1,figsize=(10,8),sharex=True,layout="constrained")
        axes[0].plot(data["time_s"],data["object_position_m"][:,2],label="object Z")
        axes[0].plot(data["time_s"],data["grasp_position_act_m"][:,2],"--",label="grasp centre Z")
        axes[0].set_ylabel("Height [m]")
        axes[1].plot(data["time_s"],data["gripper_force_command_n"],label="pushrod command")
        axes[1].plot(data["time_s"],data["grasp_normal_force_n"],label="sum pad normal forces")
        axes[1].set_ylabel("Force [N]")
        axes[2].step(data["time_s"],data["grasp_contact_sides"],where="post",label="contact sides")
        axes[2].set_ylabel("Contact sides")
        axes[2].set_yticks([0,1,2])
        phases = data["phase"]
        changes = np.r_[0,np.flatnonzero(phases[1:] != phases[:-1])+1]
        for axis in axes:
            axis.grid(alpha=.3)
            axis.legend(loc="upper left",fontsize=8)
            for index in changes:
                axis.axvline(data["time_s"][index],color="gray",alpha=.2)
        for index in changes:
            axes[0].text(data["time_s"][index],1.01,str(phases[index]),rotation=45,fontsize=7,
                         transform=axes[0].get_xaxis_transform())
        axes[-1].set_xlabel("Simulation time [s]")
        fig.savefig(directory/"grasp_contacts.png",dpi=140)
        plt.close(fig)
