# 保存的历史结果

本目录的五份原始数据文件均直接复制，没有重新计算或改写：q_table.json、train_log.csv、train_summary.json、eval_result.json、eval_baseline.json。

训练汇总记录 6000 回合、seed=0；训练末段窗口成功率 0.8067。CSV 保存逐回合奖励、成功标志、丢线、转向、步数和 ε。

两份评估 JSON 都记录 300 回合、seed=12345、成功率 1.0。eval_result.json 记录平均转向 44.513、平均步数 281.8；eval_baseline.json 记录平均转向 1.36、平均步数 236.1。只将这些数值作为原文件内容记录，不作为本次运行结果或实体车性能。

**重要：两份 JSON 的 policy 字段都写成 god_c_rule。** evaluate.py 会在默认评估路径中先把 action_fn 设为内部函数，再按其真值选标签，造成误标。根据原文档和文件命名，eval_result.json 被用作 Q-learning 结果，但缺少独立运行元数据，不能仅凭字段确认算法归属。本次未修复、未复跑或替换这两份文件。

模拟规则基线也不含完整固件的动作时序和丢线方向记忆。已有结果不足以对实车性能或算法普遍优劣作出结论。
