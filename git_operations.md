# Git 操作记录

> 记录仓库 `ahrs_test` 的分支重构操作

---

## 1. 清理被跟踪的编译产物

```bash
# 从 git 跟踪中移除所有编译生成的文件（保留本地磁盘文件）
# 匹配 Keil MDK 编译输出和 IDE 配置文件
git ls-files --cached | Where-Object { $_ -match '\.(o|d|crf|dep|hex|axf|bin|map|lst|htm|sct|uvguix|uvopt|uvoptx)$' } |
  ForEach-Object { git rm --cached $_ }

# 移除 clangd 索引缓存文件的跟踪
git ls-files --cached '.cache/' | ForEach-Object { git rm --cached $_ --quiet }

# 移除 compile_commands.json 的跟踪
git ls-files --cached | Where-Object { $_ -match 'compile_commands' } |
  ForEach-Object { git rm --cached $_ --quiet }

# 最终提交（清除跟踪记录并更新 .gitignore）
git commit -m "clean:清除跟踪的编译文件"
```

---

## 2. 创建重构分支

```bash
# 查看提交拓扑，找到含"重构"的提交
git log --all --oneline --graph --decorate

# 在 "temp：重构存档" 提交处创建 refactor 分支
git branch refactor acb508a

# 此时分支结构：
#   acb508a (refactor) ← 重构存档
#   /
#   defad4d
#   \
#   031a0ba (HEAD)     ← 当前游离节点
```

---

## 3. 将 master 重命名为 main

```bash
# 先把旧的 master 分支改名腾出位置
git branch -m master old_master

# 在当前游离节点 031a0ba 创建 main 分支
git branch main 031a0ba

# 切换到 main 分支
git checkout main

# 删除旧的 master 临时分支
git branch -D old_master
```

---

## 4. 推送到远程

```bash
# 推送新分支到 GitHub
git push origin refactor
git push origin main

# 此时远程有：main, refactor, master（三个分支）
```

---

## 5. 删除远程 master 分支

> ⚠️ 前提：需要在 GitHub 网页上将默认分支从 `master` 改为 `main`
>
> 步骤：
> 1. 打开 https://github.com/chipskiller/ahrs_test/settings/branches
> 2. "Default branch" → 点 ✎ 图标
> 3. 下拉选择 `main` → Update → 确认

```bash
# 更改默认分支后，删除远程 master
git push origin --delete master

# 清理过时的远程跟踪引用（origin/master）
git remote prune origin
```

---

## 6. 设置本地默认分支名

```bash
# 以后 git init 新建仓库时默认分支名为 main
git config --global init.defaultBranch main
```

---

## 最终分支结构

```
本地：
  * main      ← 031a0ba  clean:清除跟踪的编译文件 feat：自动上报磁力计数值
    refactor  ← acb508a  temp：重构存档

远程：
  origin/main
  origin/refactor
  ~~origin/master~~  (已删除)
```
