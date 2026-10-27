# 上传与后续维护

拟用名称：MCU-Smart-Line-Following-Car；建议可见性：Private。本次准备阶段没有创建远程仓库、提交或推送。实际账号、上传清单与本地检查结果在仓库外的待确认清单中。

用户明确批准后再创建空的私有仓库、在发布目录初始化 Git、使用用户确认的提交身份提交并推送。当前本机未检测到 gh；GitHub 插件已连接，但插件认证不等于 Git HTTPS/SSH 推送已经认证。若需要浏览器登录或 Git Credential Manager，应由用户完成，不读取或展示 Token。

后续更新仅复制原工程中明确要发布的文件到发布目录，保留原编码；先看差异、更新 README 与结果来源，检查密钥和第三方依赖，再提交。不要对原始工程运行批量同步删除。

已有远程仓库且认证完成后，在发布目录的 PowerShell 中逐步执行：

```powershell
git status --short
git diff
git add -- README.md docs firmware reinforcement-learning scripts .gitignore .gitattributes
git diff --cached --stat
git diff --cached
git commit -m "Update firmware and research documentation"
git push
```

不要使用 git add -f 提交被忽略的依赖、凭据或输出，不要强制推送。首次提交前检查 Git 身份；隐私邮箱应使用 GitHub 账号设置中本人确认的 noreply 地址，不根据账号名称猜测邮箱。若已存在同名远程库，先核对仓库与现有历史，不直接覆盖。

推送后核对远程默认分支提交 SHA、文件树与本地提交一致，确认可见性为 Private。转 Public 前再次审核所有提交历史，而不只检查最新目录。
