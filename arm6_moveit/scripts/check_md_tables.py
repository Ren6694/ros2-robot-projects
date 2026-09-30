#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Markdown 表格全量体检：列数一致 + UTF-8 + 无 CRLF。

为什么要有这个脚本（索引 00 §7 第 11 条）：批量改正则/表格时，缺陷常常落在
**没打算改**的那一行上，而且都是"呈现层出错、数值不变"——人眼读源码看不出来。
两种典型：
  1) 单元格里的竖线没转义成 \\|  => 那一行被 Markdown 拆成多一列，整表错位；
  2) 以文本模式写盘带进 CRLF    => 跨平台 diff 全行标红。

计数口径：先把行内代码 `...` 整段抹掉（代码里的 | 不是分隔符），再把转义的 \\|
换成占位符，剩下的 | 才是真分隔符。

用法：
    python check_md_tables.py [目录 ...]      # 默认当前目录的 *.md
退出码：0 = 全过；1 = 有问题（问题逐行打印）。
结果行：DOC-CHECK PASS/FAIL (N issues)
"""
import os
import re
import sys

CODE_SPAN = re.compile(r"`[^`]*`")


def cell_count(line):
    """数一行表格行的列分隔符数量（先抹掉行内代码与转义竖线）。"""
    s = CODE_SPAN.sub("\x00", line)
    s = s.replace("\\|", "\x01")
    return s.count("|")


def check_file(path):
    bad = []
    raw = open(path, "rb").read()
    try:
        text = raw.decode("utf-8")
    except UnicodeDecodeError as exc:
        return [f"{path}: NOT-UTF8 {exc}"]
    if b"\r\n" in raw:
        bad.append(f"{path}: HAS-CRLF")
    lines = text.split("\n")
    group = []
    for idx, line in enumerate(lines, 1):
        if line.lstrip().startswith("|"):
            group.append((idx, cell_count(line)))
            continue
        if len(group) > 1:
            by_count = {}
            for ln, c in group:
                by_count.setdefault(c, []).append(ln)
            if len(by_count) > 1:
                shape = ", ".join(
                    f"{c}col@L{lns[0]}..{lns[-1]}" if len(lns) > 1 else f"{c}col@L{lns[0]}"
                    for c, lns in sorted(by_count.items(), key=lambda kv: -len(kv[1]))
                )
                bad.append(f"{path}: TABLE col-mismatch in block L{group[0][0]}-{group[-1][0]} -> {shape}")
        group = []
    if len(group) > 1:
        by_count = {}
        for ln, c in group:
            by_count.setdefault(c, []).append(ln)
        if len(by_count) > 1:
            bad.append(f"{path}: TABLE col-mismatch at EOF (block L{group[0][0]}-{group[-1][0]})")
    return bad


def main():
    roots = sys.argv[1:] or ["."]
    files = []
    for root in roots:
        if os.path.isfile(root):
            files.append(root)
            continue
        for dirpath, _, names in os.walk(root):
            for n in sorted(names):
                if n.endswith(".md"):
                    files.append(os.path.join(dirpath, n))
    issues = []
    for f in sorted(files):
        issues.extend(check_file(f))
    # 只印 ASCII 标签 + 文件路径：Windows 控制台是 GBK，print 中文会抛异常，
    # 而"先写盘后打印"的脚本一旦在 print 处炸掉，文件已经改坏了（索引 §7 族 2）。
    for msg in issues:
        print(msg.encode("ascii", "backslashreplace").decode("ascii"))
    print(f"DOC-CHECK {'PASS' if not issues else 'FAIL'} files={len(files)} issues={len(issues)}")
    return 1 if issues else 0


if __name__ == "__main__":
    sys.exit(main())
