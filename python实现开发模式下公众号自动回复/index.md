# Python实现开发模式下公众号自动回复


Tags: #python #微信公众号 #自动回复  
Links: [[Python下web.py使用ssl加密]], [[Python下web.py的环境部署]], [[可点击式文字菜单]]   

---

# Python实现开发模式下公众号自动回复

## 关注公众号自动回复

如果一个用户关注了一个公众号，公众号后台会收到对应的XML消息。
``` xml
<xml>
<ToUserName><![CDATA[公众号]]></ToUserName>
<FromUserName><![CDATA[粉丝号]]></FromUserName>
<CreateTime>123456789</CreateTime>
<MsgType><![CDATA[event]]></MsgType>
<Event><![CDATA[subscribe]]></Event>
</xml>
```
以下是参数介绍:
![](Attachments/Pasted%20image%2020201017163846%20subscription%20parameters.png)

因此，我们可以首先解析这个XML，获得MsgType为event，然后判断该event是否为"subscribe"，如果是，则进行回复消息。
handle.py中对应自动回复关注:
``` python
class Handle(object):
    def GET(self):
	    ...
	
	def POST(self):
	    ...
		if recMsg.MsgType == 'event':
		    if recMsg.Event == 'subscribe':
			    content = u"欢迎关注".encode('utf-8')
				replyMsg = reply.TextMsg(toUser, fromUser, content)
				return replyMsg.send()
```


## 根据关键词自动回复
如果用户给公众号发送一段text消息，比如"hello"，那么后台收到的xml就是:
``` xml
<xml>
<ToUserName><![CDATA[公众号]]></ToUserName>
<FromUserName><![CDATA[粉丝号]]></FromUserName>
<CreateTime>1460541339</CreateTime>
<MsgType><![CDATA[text]]></MsgType>
<Content><![CDATA[hello]]></Content>
</xml>
```
其中MsgType有"text"、"image"、"voice"、"video"、"link"、"event"、"location"等多种类型。
公众号给用户发消息也是同理，公众号向用户发送一个xml，只是发件人和收件人对调，内容自定。
``` xml
<xml>
<ToUserName><![CDATA[粉丝号]]></ToUserName>
<FromUserName><![CDATA[公众号]]></FromUserName>
<CreateTime>1460541339</CreateTime>
<MsgType><![CDATA[text]]></MsgType>
<Content><![CDATA[test]]></Content>
</xml>
```
接下来，在已有receive.py和reply.py的前提下，我们可以通过修改handle.py来自定义公众号如何进行自动回复。
handle.py中对应部分
``` python
class Handle(object):
    def GET(self):
	    ...

    def POST(self):
	    ...
		    if recMsg.MsgType == 'text':
				content = automaticReply(recMsg.Content)
			    # 判断接受的信息是否包含预设的关键词交由一个独立的方法来处理
				replyMsg = reply.TextMsg(toUser, fromUser, content)
				return replyMsg.send()
```

*ps*: python中字符串前加u以确保后续字符串以unicode进行编码，一般用于中文字符串前，防止因源码储存问题出现的乱码。但不需要对字符串再使用encode('utf-8‘),不然就会变成b'\xe3\xe2...'的格式。
