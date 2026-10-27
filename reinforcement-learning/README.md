# Q-learning 模拟实验

src/、tests/ 和 tools/ 中的代码均按原字节复制，未修改算法、训练参数或评估实现。状态 16、动作 6，使用标准库实现 Q-learning；requirements.txt 仅记录环境说明，无需 pip 安装。

运行方式、训练输出覆盖行为和局限见根目录 README。原有 tests/test_q_learning.py 中的 TestTrainingRuns 会启动短程训练；本次整理没有运行完整测试，也没有运行演示脚本（演示也会更新内存 Q 表）。

结果、策略导出与固件是独立模块。当前没有集成查表控制，也没有在实体车验证。原代码文档中的 Ant/god.c 路径是原始布局的历史引用，发布副本对应 firmware/Ant/god.c；不应按过时行号修改固件。
