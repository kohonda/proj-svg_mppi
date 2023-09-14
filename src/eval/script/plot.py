from typing import List, Dict
import os
import fire
import pandas as pd
from matplotlib import pyplot as plt
import seaborn as sns

# to avoid type3 font
plt.rcParams["pdf.fonttype"] = 42
plt.rcParams["ps.fonttype"] = 42


def read_files(result_dir: str) -> Dict[str, pd.DataFrame]:
    # get all csv path and name in result_dir
    files = os.listdir(result_dir)
    csv_files = [file for file in files if file.endswith(".csv")]

    csv_paths = [os.path.join(result_dir, csv_path) for csv_path in csv_files]
    csv_names = [os.path.splitext(csv_path)[0] for csv_path in csv_files]

    data_dict = {}
    for csv_path, csv_name in zip(csv_paths, csv_names):
        data_dict[csv_name] = pd.read_csv(csv_path)

    return data_dict


def output_result(result_dir: str) -> None:
    data_dict = read_files(result_dir)
    OBSTACLE_NUM_PER_TRIAL = 5
    data_collision = pd.DataFrame()
    data_success = pd.DataFrame()
    data_mean_calc_time = pd.DataFrame()
    data_max_calc_time = pd.DataFrame()
    data_min_calc_time = pd.DataFrame()
    for key in data_dict.keys():
        collision_num = sum(data_dict[key]["collision"])
        sum_obs_num = len(data_dict[key]["collision"]) * OBSTACLE_NUM_PER_TRIAL
        collision_rate = collision_num / sum_obs_num * 100
        data_collision = pd.concat(
            [data_collision, pd.DataFrame([collision_rate], columns=[key])], axis=1
        )

        success = sum(data_dict[key]["success"])
        success_rate = success / len(data_dict[key]["success"]) * 100
        data_success = pd.concat(
            [data_success, pd.DataFrame([success_rate], columns=[key])], axis=1
        )

        mean_calc_time = data_dict[key]["mean_calculation_time"].mean()
        data_mean_calc_time = pd.concat(
            [data_mean_calc_time, pd.DataFrame([mean_calc_time], columns=[key])], axis=1
        )

        max_calc_time = data_dict[key]["max_calculation_time"].max()
        data_max_calc_time = pd.concat(
            [data_max_calc_time, pd.DataFrame([max_calc_time], columns=[key])], axis=1
        )

        min_calc_time = data_dict[key]["min_calculation_time"].min()
        data_min_calc_time = pd.concat(
            [data_min_calc_time, pd.DataFrame([min_calc_time], columns=[key])], axis=1
        )

    # save result as txt
    with open(os.path.join(result_dir, "result.txt"), "w") as f:
        f.write("===== collision rate (%) =====\n")
        f.write(data_collision.to_string(index=False))
        f.write("\n\n")
        f.write("===== success rate (%) =====\n")
        f.write(data_success.to_string(index=False))
        f.write("\n\n")
        f.write("===== mean calculation time (ms) =====\n")
        f.write(data_mean_calc_time.to_string(index=False))
        f.write("\n\n")
        f.write("===== max calculation time (ms) =====\n")
        f.write(data_max_calc_time.to_string(index=False))
        f.write("\n\n")
        f.write("===== min calculation time (ms) =====\n")
        f.write(data_min_calc_time.to_string(index=False))
        f.write("\n\n")


def single_cost_box(result_dir: str, ablation_study: str = None) -> None:
    data_dict = read_files(result_dir)
    data = pd.DataFrame()

    # print result
    output_result(result_dir)

    # plot cost
    for key in data_dict.keys():
        data = pd.concat(
            [
                data,
                data_dict[key]["state_cost"] + data_dict[key]["collision_cost"],
            ],
            axis=1,
        )
    data.columns = [key for key in data_dict.keys()]

    # IQR removal
    # Sometimes the simulator glitches on some trials and you end up dizzy in the wall.
    # Q1 = data.quantile(0.25)
    # Q3 = data.quantile(0.75)
    # IQR = Q3 - Q1
    # data = data[~((data < (Q1 - 1.5 * IQR)) | (data > (Q3 + 1.5 * IQR))).any(axis=1)]

    if ablation_study is None:
        # reordering
        # For comparison with baselines
        align_data = pd.DataFrame(
            {
                "MPPI\n(cov:0.025)": data["forward_mppi(cov:0.025)"],
                # "forward_mppi\n(cov:0.05)": data["forward_mppi(cov:0.05)"],
                "MPPI\n(cov:0.075)": data["forward_mppi(cov:0.075)"],
                "MPPI\n(cov:0.1)": data["forward_mppi(cov:0.1)"],
                "Reverse-MPPI": data["reverse_mppi"],
                "SV-MPC": data["sv_mpc"],
                "SVG-MPPI\n(Ours)": data["svg_mppi"],
            }
        )
    elif ablation_study == "cov":
        # For ablation study
        align_data = pd.DataFrame(
            {
                # "cov:0.001": data["cov:0.001"],
                "0.01": data["cov:0.01"],
                "0.015": data["cov:0.015"],
                "0.02": data["cov:0.02"],
                "0.025": data["cov:0.025"],
                "0.03": data["cov:0.03"],
                "0.05": data["cov:0.05"],
                "0.075": data["cov:0.075"],
                "0.1": data["cov:0.1"],
                "adaptive\n(SVG-MPPI)": data["cov:adaptive"],
            }
        )
    elif ablation_study == "nom":
        align_data = pd.DataFrame(
            {
                "wo_nominal_solution": data["wo_nominal_solution"],
                "w_nominal_solution": data["w_nominal_solution"],
            }
        )

    means = align_data.mean()

    plt.figure(figsize=(20, 12))

    sns.set_theme(style="whitegrid")
    sns.set_context("poster")
    sns.set_style("ticks")
    sns.set_palette("Set2")
    sns.boxplot(
        data=align_data,
        showmeans=True,
        meanprops={
            "marker": "o",
            "markerfacecolor": "black",
            "markeredgecolor": "black",
            "markersize": 10,
        },
        showfliers=False,
    )

    # plot mean
    for i, mean in enumerate(means):
        # plt.scatter(i, mean, marker="o", color="black", s=50)
        plt.text(
            i,
            mean + 0.01,
            # mean + 0.15,
            "{:.2f}".format(mean),
            ha="right",
            va="center",
            fontsize=25,
        )

    # plt.yscale("log")
    if ablation_study == "cov":
        plt.xlabel("Covariance of steering angle [rad]", fontsize=30)
    else:
        plt.xlabel("Algorithm", fontsize=30)
    plt.ylabel("Mean trajectory state cost per trial", fontsize=30)

    # save as pdf
    path = os.path.join(result_dir, "cost_box.pdf")
    plt.savefig(path, bbox_inches="tight")

    plt.show()


def double_cost_box(
    path_tracking_result_dir: str, obstacle_avoidance_result_dir: str, save_dir: str
) -> None:
    pt_data_dict = read_files(path_tracking_result_dir)
    oa_data_dict = read_files(obstacle_avoidance_result_dir)

    # print result
    output_result(path_tracking_result_dir)
    output_result(obstacle_avoidance_result_dir)

    # plot cost
    pt_data = pd.DataFrame()
    for key in pt_data_dict.keys():
        pt_data = pd.concat(
            [
                pt_data,
                pt_data_dict[key]["state_cost"] + pt_data_dict[key]["collision_cost"],
            ],
            axis=1,
        )
    pt_data.columns = [key for key in pt_data_dict.keys()]

    oa_data = pd.DataFrame()
    for key in oa_data_dict.keys():
        oa_data = pd.concat(
            [
                oa_data,
                oa_data_dict[key]["state_cost"] + oa_data_dict[key]["collision_cost"],
            ],
            axis=1,
        )
    oa_data.columns = [key for key in oa_data_dict.keys()]

    # reordering
    # For comparison with baselines
    pt_align_data = pd.DataFrame(
        {
            "MPPI\n(cov:0.025)": pt_data["forward_mppi(cov:0.025)"],
            "MPPI\n(cov:0.075)": pt_data["forward_mppi(cov:0.075)"],
            "MPPI\n(cov:0.1)": pt_data["forward_mppi(cov:0.1)"],
            "Reverse-MPPI": pt_data["reverse_mppi"],
            "SV-MPC": pt_data["sv_mpc"],
            "SVG-MPPI\n(Ours)": pt_data["svg_mppi"],
        }
    )
    pt_means = pt_align_data.mean()

    oa_align_data = pd.DataFrame(
        {
            "MPPI\n(cov:0.025)": oa_data["forward_mppi(cov:0.025)"],
            "MPPI\n(cov:0.075)": oa_data["forward_mppi(cov:0.075)"],
            "MPPI\n(cov:0.1)": oa_data["forward_mppi(cov:0.1)"],
            "Reverse-MPPI": oa_data["reverse_mppi"],
            "SV-MPC": oa_data["sv_mpc"],
            "SVG-MPPI\n(Ours)": oa_data["svg_mppi"],
        }
    )
    oa_means = oa_align_data.mean()

    # align like seaborn dataset tips format
    pt_align_data = pd.melt(pt_align_data, var_name="Algorithm", value_name="Cost (PT)")
    oa_align_data = pd.melt(oa_align_data, var_name="Algorithm", value_name="Cost (OA)")

    # pt/oaの列を追加
    align_data = pd.concat([pt_align_data, oa_align_data], axis=0)
    align_data["Scenario"] = ["PT"] * len(pt_align_data) + ["OA"] * len(oa_align_data)

    fig, ax1 = plt.subplots(figsize=(30, 12))

    sns.set_theme(style="whitegrid")
    sns.set_context("poster")
    sns.set_style("ticks")
    sns.set_palette("Set2")

    sns.boxplot(
        x="Algorithm",
        y="Cost (PT)",
        hue="Scenario",
        data=align_data,
        showmeans=True,
        meanprops={
            "marker": "o",
            "markerfacecolor": "black",
            "markeredgecolor": "black",
            "markersize": 10,
        },
        showfliers=False,
        ax=ax1,
    )

    ax2 = ax1.twinx()
    sns.boxplot(
        x="Algorithm",
        y="Cost (OA)",
        hue="Scenario",
        data=align_data,
        showmeans=True,
        meanprops={
            "marker": "o",
            "markerfacecolor": "black",
            "markeredgecolor": "black",
            "markersize": 10,
        },
        showfliers=False,
        ax=ax2,
    )

    # delete x label
    ax1.set_xlabel("")
    ax2.set_xlabel("")

    # set legend size
    handles, labels = ax1.get_legend_handles_labels()
    ax1.legend(handles=handles[0:6], labels=labels[0:6], fontsize=40, loc="upper right")
    ax2.legend(
        handles=handles[6:12], labels=labels[6:12], fontsize=40, loc="upper right"
    )

    ax2.set_ylim(0, 55.0)
    ax1.set_ylim(0, 6.5)

    # ax1.set_xlabel("Algorithm", fontsize=30)
    ax1.set_ylabel("Mean sequence state cost per lap (PT)", fontsize=40)
    ax2.set_ylabel("Mean sequence state cost per lap (OA)", fontsize=40)

    ax1.tick_params(labelsize=45)
    ax2.tick_params(labelsize=45)

    # text mean
    for i, mean in enumerate(pt_means):
        ax1.text(
            i,
            mean + 0.15,
            "{:.2f}".format(mean),
            ha="right",
            va="center",
            fontsize=45,
        )
    for i, mean in enumerate(oa_means):
        ax2.text(
            i,
            mean + 1.5,
            "{:.2f}".format(mean),
            ha="left",
            va="center",
            fontsize=45,
        )

    # save as pdf
    path = os.path.join(save_dir, "comparison_with_baselines.pdf")
    plt.savefig(path, bbox_inches="tight")

    plt.show()


if __name__ == "__main__":
    fire.Fire(double_cost_box)
