# Tips

## Using snippets as File Template
- Install vscode ext. [Auto Snippet](ext install Gruntfuggly.auto-snippet)
- Create snippet to run on file created
- Config vscode 
- Create snippet [online](https://snippet-generator.app/)


```json title="vscode settings"
autoSnippet.snippets: [
    { "pattern": "**/model.config", "snippet": "gazebo_model_config" },
    { "language": "javascript", "snippet": "example", "commands":[ "editor.action.commentLine" ] }
]
```

```json title="gazebo model config file template"
"gazebo_model_config": {
		"prefix": "gz_config",
		"body": [
		  "<?xml version=\"1.0\"?>",
		  "<model>",
		  "  <name>${1}</name>",
		  "  <version>1.0</version>",
		  "  <sdf version=\"${2|1.5,1.6|}\">${3:${1}}.sdf</sdf>",
		  "  <author>",
		  "    <name></name>",
		  "    <email></email>",
		  "  </author>",
		  "  <description>",
		  "  </description>",
		  "</model>"
		],
		"description": "gazebo model config file template"
	  }
```

---

## Run on Save
[ext install emeraldwalk.RunOnSave](https://marketplace.visualstudio.com/items?itemName=emeraldwalk.RunOnSave)
[variables-reference](https://code.visualstudio.com/docs/editor/variables-reference)

```
${fileBasenameNoExt} : file name without extension
```

!!! Note
	`fileBasenameNoExt` replace `fileBasenameNoExtension`

```
"emeraldwalk.runonsave": {
	"commands": [
		{
			"match": "\\.sdf.xaco$",
			"cmd": "export PYTHONPATH=/opt/ros/foxy/lib/python3.8/site-packages && /opt/ros/foxy/bin/xacro ${file} -o ${fileDirname}/${fileBasenameNoExt}"
		}
	]
}
```

---

# File Associations
Assign file ext to language  
for example gazebo `sdf` and `world` files to `xml` language

```json title="settings.json"
"files.associations": {
        "*.sdf": "xml",
        "*.world": "xml",
    }
```