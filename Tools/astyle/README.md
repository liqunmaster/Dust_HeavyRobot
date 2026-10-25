# PX4 格式基准

采用 PX4 v1.16.0 的 .editorconfig 和 Tools/astyle/astylerc：Tab、宽度 8、Linux 大括号、LF，AStyle 最大代码行宽为 120。根 .astylerc 和本目录 astylerc 相同，clang-format 只作近似支持。

`public:`、`protected:` 和 `private:` 与类声明左边界对齐，成员缩进一个 Tab；类和函数的大括号另起一行，结构体、枚举及联合体的左大括号跟随声明。排版以官方 AStyle 配置为准。VS Code 的 C/C++ 保存和输入自动格式化已关闭，使用 `format_px4` 任务统一排版，避免 clangd 的近似规则覆盖格式。

自有代码的说明注释使用中文，保留标识符、Doxygen 标签和必要的技术缩写；上游版权和许可证正文保持原文。格式来源：[PX4 v1.16.0 astylerc](https://github.com/PX4/PX4-Autopilot/blob/v1.16.0/Tools/astyle/astylerc)。

本轮 AStyle 3.1 从 Debian 官方源码镜像构建：

- 源码：<https://deb.debian.org/debian/pool/main/a/astyle/astyle_3.1.orig.tar.gz>
- SHA-256：cbcc4cf996294534bb56f025d6f199ebfde81aa4c271ccbd5ee1c1a3192745d7

安装在本工程 build/tools，不改系统环境：

```powershell
python Tools/astyle/bootstrap.py --cxx D:/mingw64/bin/g++.exe
python Tools/format_code.py
python Tools/format_code.py --check
```

可用 --astyle 指定已安装的 AStyle 3.1。仅格式化 src、platforms、boards、test 的自有 C/C++，ThirdParty、sim、SDK 和生成文件不参与。
