# Python下web.py使用ssl加密


Tags: #python #webpy #ssl #https  
Links: [[Python下web.py的环境部署]], [[Python实现开发模式下公众号自动回复]]

---

# Python下web.py使用ssl加密

## [[https]]，ssl与[[443端口]]：
一般默认采用https://开头时是使用的443端口。而如果要使用https://就必须要配置ssl证书。

## [[ssl]]证书：
可以利用openssl自签发ssl证书，也可以去阿里云或其他网络服务提供商购买收费型证书。没有可信的ssl证书，即使采用了https协议，也不会外来访问源认可。

- openssl自签发证书举例：
首先通过如下shell命令，回答问题，设置密码生成证书。
``` shell
openssl genrsa -des3 -out server.key 1024
openssl req -new -key server.key -out server.csr
openssl x509 -req -days 365 -in server.csr -signkey server.key -out server.crt
```
然后在有证书的文件夹下，通过如下命令来避免执行程序时输入密码的操作：
``` shell
openssl rsa -in server.key -out server.key
```
> pem, crt: 为证书
> key: 为私钥

## 绑定[[IP]]：
阿里云收费型ssl证书才提供绑定ip的功能，不需要绑定域名，在对接三方API时，可以直接使用IP作url访问。

## 绑定[[域名]]：
只能绑定域名的ssl证书，在部署后，还需要为该域名进行单独解析，解析到自己想要的IP地址上，然后才可以通过访问https://域名，代替直接访问https://IP。只需要设置域名解析，不需要再配置web程序。

## [[web.py]]下使用ssl：
``` python
import web
from handle import Handle
from cheroot.server import HTTPServer
from cheroot.ssl.builtin import BuiltinSSLAdapter

HTTPServer.ssl_adapter = BuiltinSSLAdapter(
    certificate='/www/server.pem',
    private_key='/www/server.key')
urls = (
    '/wx', 'Handle',
)

if __name__ == '__main__':
    app = web.application(urls, globals())
    app.run()
```
必须添加HTTPServer和BuiltinSSLAdapter包，然后填入自己的ssl证书和私钥路径。然后后台执行命令`python3 main.py 443`来启动程序。
