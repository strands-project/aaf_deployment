import requests
from json import loads
from re import match
from pprint import pprint


class FacebookNews:

    def __init__(self, app_id='548519798650746',
                 page_id='158245387541011',
                 app_secret=''):
        self.app_id = app_id
        self.app_secret = app_secret
        self.page_id = page_id
        self.url_token_req = 'https://graph.facebook.com/oauth/access_token?client_id=%s&client_secret=%s&grant_type=client_credentials'
        self.url_page_req = 'https://graph.facebook.com/v2.2/%s/feed?access_token=%s'
        self.url_post_req = 'https://graph.facebook.com/v2.2/%s?access_token=%s'
        self.access_token = None
        self.access_token_pattern = 'access_token=(.+)'

    def login(self):
        try:
            t = requests.get(self.url_token_req
                             % (self.app_id, self.app_secret)).text
            m = match(self.access_token_pattern, t)
            self.access_token = m.group(1)
        except Exception as e:
            print e

    def get(self):
        try:
            t = requests.get(self.url_page_req
                             % (self.page_id, self.access_token)).text
            posts = loads(t)
#            for post in posts['data']:
#                t = requests.get(self.url_post_req
#                                 % (post['id'], self.access_token)).text
#                pprint(loads(t))

            return posts
        except Exception as e:
            print e
        return None

if __name__ == "__main__":
    # the APP secret is to be obtained from https://developers.facebook.com/apps/548519798650746/dashboard/
    fb = FacebookNews(app_secret='XXXX')
    
    fb.login()
    pprint(fb.get())
    
    print fb.access_token

