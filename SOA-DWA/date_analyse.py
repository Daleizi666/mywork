import numpy as np
from scipy.stats import f_oneway, ttest_ind

np.random.seed(42)

# 模拟数据 + 检验函数
def monte_carlo_statistical_test(data_summary, sample_size=30):
    results = []
    for scene_info in data_summary:
        scene_name = scene_info["scene"]
        metric_name = scene_info["metric"]
        groups = scene_info["groups"]

        # 重建数据
        samples = {}
        for algo, stats in groups.items():
            mean, std = stats
            samples[algo] = np.random.normal(mean, std, sample_size)

        # ANOVA
        f_stat, p_anova = f_oneway(*samples.values()) if len(samples) > 2 else (None, None)

        # t 检验
        t_test_results = []
        algos = list(samples.keys())
        for i in range(len(algos)):
            for j in range(i + 1, len(algos)):
                a1, a2 = algos[i], algos[j]
                t_stat, p_val = ttest_ind(samples[a1], samples[a2], equal_var=False)
                t_test_results.append({
                    "算法1": a1,
                    "算法2": a2,
                    "t值": round(t_stat, 3),
                    "p值": round(p_val, 4)
                })

        results.append({
            "场景": scene_name,
            "指标": metric_name,
            "ANOVA p值": round(p_anova, 4) if p_anova is not None else "N/A",
            "t检验结果": t_test_results
        })

    return results

# 示例输入数据（你可以根据自己数据扩展）
data_input = [
    {
        "scene": "Static Known Scenario - Scenario 1",
        "metric": "Path Length",
        "groups": {
            
        }
    },
    {
        "scene": "Static Known Scenario - Scenario 2",
        "metric": "Path Length",
        "groups": {
            
        }
    },
    {
        "scene": "Dynamic Unknown Scenario - Scenario 2",
        "metric": "Path Length",
        "groups": {
          
        }
    }
]

# 执行函数
results = monte_carlo_statistical_test(data_input)

# 打印结果
for res in results:
    print(f"\n场景: {res['场景']} | 指标: {res['指标']}")
    print("ANOVA P值:", res["ANOVA p值"])
    for t in res["t检验结果"]:
        print(f"  {t['算法1']} vs {t['算法2']} -> t={t['t值']}, p={t['p值']}")
