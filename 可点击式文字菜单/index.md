# 微信公众号可点击式文字菜单


Tags: #微信公众号 #自动回复 #文字菜单  
Links: [[Python实现开发模式下公众号自动回复]]

---

# 微信公众号可点击式文字菜单

基于微信浏览器的URL Schemes来实现对微信内部消息的控制，weixin://。
基本格式如下：
``` url
weixin://bizmsgmenu?msgmenucontent=这里写菜单内容&msgmenuid=这里写菜单ID
```
使用方法如下：
``` xml
<a href="weixin://bizmsgmenu?msgmenucontent=充值&msgmenuid=1">充值</a>
```
当用户点击该超链接生成的蓝色可互动文字时，客户端将代替用户自动向微信服务器发送指定的文字消息
