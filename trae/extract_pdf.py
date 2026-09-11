# -*- coding: utf-8 -*-
"""提取 B 题和附件的文本内容,便于后续分析"""
import os
import sys

import pdfplumber

PDF_PATH = r"d:\数学建模\B题\B题.pdf"
OUT_PATH = r"d:\数学建模\B题\B题_文本.txt"


def main():
    if not os.path.exists(PDF_PATH):
        print(f"PDF 不存在: {PDF_PATH}")
        sys.exit(1)

    with pdfplumber.open(PDF_PATH) as pdf, open(OUT_PATH, "w", encoding="utf-8") as fout:
        for i, page in enumerate(pdf.pages, 1):
            text = page.extract_text() or ""
            fout.write(f"\n===== 第 {i} 页 =====\n")
            fout.write(text)
            fout.write("\n")
            print(f"第 {i} 页: {len(text)} 字符")

    size = os.path.getsize(OUT_PATH)
    print(f"输出文件: {OUT_PATH}  大小: {size} 字节")


if __name__ == "__main__":
    main()
