---
title: mkdocs setup and config
tags:
    - mkdocs
    - tags
    - plugins
---
# mkdocs

## Tags
Using [plugins](https://github.com/jldiaz/mkdocs-plugin-tags)

!!! Note
    Material tags plugin work only for insiders

### Usage
```
---
title: hello tags
tags:
 - tag1
 - tag2
---
```

```title="mkdocs.yaml"
plugins:
  - tags
```