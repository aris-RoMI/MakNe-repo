import re

text = "@jigu0825 fejksei"
result = re.findall(r"@(\w+)", text)

print(result)