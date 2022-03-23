# nginx反向代理(端口转发)


Tags: #nginx #反向代理 #nginx  
Links:

---

# nginx反向代理(端口转发)

[[nginx]]可以在监听一个端口时，将消息按配置文件转发至其他内部端口，以实现多个应用部署在同一个端口上。

``` nginx
server {
    listen       80;  #打算使用的对外统一端口
	server_name  localhost;  #外界可以访问的域名
	root         /usr/share/nginx/html;  #主页面的路径
	index        index.html;  #主页面的名字

    #path是域名下的路径
    location /path {  
        proxy_pass http://localhost:8080;  #代理服务器，比如动态应用程序站点
	    ...(proxy_set_header);  
		#重写头信息并将之一并转发到代理服务器，以确保代理服务器获得正确的发送方的信息
    }
}
```

以下是一个完整的代码示例：
``` nginx
server {
    listen       80 default_server;
    listen       [::]:80 default_server;
    server_name  localhost;
    root         /usr/share/nginx/html/;
	index        index.html;

    location /path {
        add_header Cache-Control no-cache;
        proxy_set_header   Host $host:$server_port;
        proxy_set_header   X-Forwarded-For  $proxy_add_x_forwarded_for;
        proxy_set_header   X-Real-IP        $remote_addr;
        proxy_connect_timeout 30s;
        proxy_pass       http://localhost:8080;
    }

    error_page 404 /404.html;
    location = /404.html {
    }

    error_page 500 502 503 504 /50x.html;
    location = /50x.html {
    }
}
```
