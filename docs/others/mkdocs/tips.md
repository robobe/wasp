---
title: mkdocs-material
tags:
    - material
    - tips
---

## Images
- [material images](https://squidfunk.github.io/mkdocs-material/reference/images/#image-alignment)

```title="demo"
![Image title](https://dummyimage.com/600x400/eee/aaa){ align=left width=100}
```

### snippet
```json title="mk_image_caption"
"mk_image_caption": {
        "prefix": "mk_img_cap",
        "body": [
          "<figure markdown>",
          "![](${1:image path})",
          "<figcaption>",
          "${2:caption}",
          "</figcaption>",
          "</figure>",
          "\n"
        ],
        "description": "insert image with captions"
    }
```